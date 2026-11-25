"""Read the archived SITL vehicle_local_position instance 0 without third-party packages."""
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import tarfile

from collect_heading import require

FIELDS = ("timestamp", "timestamp_sample", "heading_good_for_control", "heading_reset_counter", "delta_heading")
TYPES = {"int8_t":"b","uint8_t":"B","int16_t":"h","uint16_t":"H","int32_t":"i","uint32_t":"I",
         "int64_t":"q","uint64_t":"Q","char":"c","bool":"?","float":"f","double":"d"}


def layout(format_text):
    offsets = {}
    where = 0
    for declaration in format_text.split(";"):
        if not declaration:
            continue
        typ, field = declaration.split(" ")
        name, count = re.fullmatch(r"(\w+)(?:\[(\d+)\])?",typ).groups()
        decoder = struct.Struct("<"+(count or "")+TYPES[name])
        offsets[field]=(where,decoder)
        where += decoder.size
    return [offsets[key] for key in FIELDS]


def read_positions(data):
    # This auditor implements just the single, primitive-only layout used in these closed logs.
    # The full pyulog reader is used separately by collect_heading.py.
    require(data[:7]==b'ULog\x01\x12\x35',"Invalid ULog header")
    cursor, topic, fmt, subscription = 16, "vehicle_local_position", None, None
    points=[]
    while cursor < len(data):
        size, kind = struct.unpack_from("<HB",data,cursor)
        begin = cursor+3
        cursor = begin+size
        require(cursor<=len(data),"Incomplete record in a closed ULog")
        if kind==ord("B"):
            require(not any(data[begin+8:begin+16]),"Unsupported appended/incompatible log")
        elif kind==ord("F"):
            name, body=data[begin:cursor].split(b":",1)
            if name==topic.encode():
                fmt=layout(body.decode("ascii"))
        elif kind==ord("A"):
            instance, msgid=struct.unpack_from("<BH",data,begin)
            if instance==0 and data[begin+3:cursor]==topic.encode():
                require(fmt is not None,"Missing format")
                subscription=msgid
        elif kind==ord("R") and struct.unpack_from("<H",data,begin)[0]==subscription:
            subscription=None
        elif kind==ord("D") and struct.unpack_from("<H",data,begin)[0]==subscription:
            values=tuple(decoder.unpack_from(data,begin+2+offset)[0] for offset,decoder in fmt)
            points.append(values)
    require(len(points)>1 and all(p[0]<=q[0] for p,q in zip(points,points[1:])),"Invalid position series")
    return points


def independent_gate(points, indices, request_time):
    # Work in integer microseconds, walk backward from the last published sample.
    t=round(request_time*1_000_000)
    i=bisect_right(indices,t)-1
    if i<0 or not points[i][2]:
        return False
    if not -20_000 <= t-points[i][1] <= 650_000:
        return False
    first=i
    while (first>0 and points[first-1][2] and points[first-1][3]==points[i][3]
           and 0<=points[first][1]-points[first-1][1]<=650_000):
        first-=1
    return points[i][1]-points[first][1]>=1_000_000


def read_case(data, report, version):
    require(hashlib.sha256(data).hexdigest()==report["ulg_sha256"],"Archived member checksum differs")
    positions=read_positions(data)
    indices=[r[0] for r in positions]
    first_good_s=next((r[0]/1e6 for r in positions if r[2]),None)
    resets=[dict(s=row[0]/1e6, delta_deg=math.degrees(row[4]),counter=row[3])
            for prev,row in zip(positions,positions[1:]) if prev[3]!=row[3]]
    require(first_good_s==report["heading_good_first_s"],"Onboard validity timestamps disagree")
    if version=="V04":
        reqs=[(r["request_s"],r.get("frame_s")) for r in report["requests"]]
        reference=[(r["stamp"],r["counter"],r["delta_deg"]) for r in report["resets"]]
        require(len(reference)==len(resets) and all(t==r["s"] and c==r["counter"] and
                abs(d-r["delta_deg"])<.00001 for (t,c,d),r in zip(reference,resets)),"Heading resets disagree")
    else:
        reqs=[(r["request_sim"],None) for r in report["gate_samples"]]
    request_pass=[independent_gate(positions,indices,t) for t,_ in reqs]
    frame_pass=[independent_gate(positions,indices,f) for _,f in reqs if f is not None and f>0]
    first=min((t for t,_ in reqs),default=None)
    late=[r for r in resets if first is not None and r["s"]>first]
    good_request=bool(reqs and all(request_pass) and not late)
    if version=="V04":
        require(good_request==report["gate_verified_all_requests"],"Request-time verdict disagrees across parsers")
        require(request_pass==[r["request_gate"]["pass_gate"] for r in report["requests"]],
                "Per-request verdict disagrees across parsers")
        require(frame_pass==[r["frame_gate"]["pass_gate"] for r in report["requests"] if r["frame_gate"]],
                "Per-frame verdict disagrees across parsers")
    return dict(case=report["case"],member_sha256=report["ulg_sha256"],samples=len(positions),
                first_good_s=first_good_s, first_request_s=first, late_resets=late, heading_resets=resets,
                request_count=len(reqs),frame_count=len(frame_pass),valid_requests=sum(request_pass),
                valid_frames=sum(frame_pass),gate_all=good_request,expected_source_gate=(
                  report["gate_verified_all_requests"] if version=="V04" else report["pass_heading_gate"]))


def replay(root, version):
    if version=="V03":
        records=json.loads((Path(__file__).resolve().parent.parent/
                           "heading-reset/v03-yaw-events.json").read_text(encoding="utf-8-sig"))
    else:
        records=json.loads((Path(__file__).resolve().parent/"coldstart-heading.json")
                           .read_text(encoding="utf-8-sig"))["cases"]
    table={r["case"]:r for r in records}
    seen=set()
    results=[]
    for archive in sorted(root.glob(version+"-px4-ulogs-*.tar.gz")):
        with tarfile.open(archive,"r|gz") as tar:
            for member in tar:
                case=member.name.split("/",1)[0]
                require(case in table and case not in seen and member.isfile(), "Unexpected archive entry")
                require(member.name==case+"/"+Path(table[case]["ulg"]).name,"Wrong mapped member")
                with tar.extractfile(member) as stream:
                    results.append(read_case(stream.read(),table[case],version))
                seen.add(case)
    require(seen==set(table) and len(seen)==16,"Incomplete archive set")
    return sorted(results,key=lambda r:r["case"])


if __name__=="__main__":
    require(len(sys.argv)==3 and sys.argv[2] in ("V03","V04"),"Usage: replay_archive.py archive_dir V03|V04")
    print(json.dumps(replay(Path(sys.argv[1]),sys.argv[2]),indent=2,allow_nan=False))

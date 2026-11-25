#!/usr/bin/env python3
"""Compile the EGO checkout's filtered projection loop and check its ray endpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

PRELUDE = """
#include <cstdint>
#include <iostream>
#include <vector>
#include <iomanip>
struct DepthImage {
  std::vector<uint16_t> pixels;
  template<class T> T *ptr(int) {return pixels.data();}
};
struct Params {
  int depth_filter_margin_ = 1, skip_pixel_ = 2;
  double max_ray_length_ = 5.0, depth_filter_mindist_ = .2, depth_filter_maxdist_ = 5.;
} mp_;
struct Data {DepthImage depth_image_;} md_;
int main() {
  // Sample columns 1, 3, 5. Padding at 7 has its own zero return.
  md_.depth_image_.pixels = {0,1000,0,0,0,2300,0,0};
  uint16_t *row_ptr;
  double depth;
  const int v = 0, cols = md_.depth_image_.pixels.size();
  const double inv_factor = .001;
"""
END = """
    std::cout << u << " " << std::setprecision(8) << depth << "\\n";
  }
}
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path, help='grid_map.cpp in the checkout used for the build')
    ap.add_argument('--out', type=Path)
    ap.add_argument('--compiler', default='g++')
    args = ap.parse_args()
    source = args.source.read_bytes()
    text = source.decode('utf-8')
    start = text.index('row_ptr =', text.index('const double inv_factor'))
    end = text.index('// project to world frame', start)
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        cpp = folder / 'sample.cpp'
        exe = folder / 'sample.exe'
        cpp.write_text(PRELUDE + text[start:end] + END, encoding='utf-8')
        subprocess.run([args.compiler, '-std=c++11', str(cpp), '-o', str(exe)], check=True)
        lines = subprocess.check_output([str(exe)], universal_newlines=True).splitlines()
    expected = {1: 1.0, 3: 5.1, 5: 2.3}
    observed = {int(col): float(value) for col, value in (line.split() for line in lines)}
    report = dict(passed=observed == expected, projected_depth_m=observed,
                  expected_depth_m=expected, input_depth_mm=[0,1000,0,0,0,2300,0,0],
                  source_sha256=hashlib.sha256(source).hexdigest())
    rendered = json.dumps(report, indent=2) + '\n'
    print(rendered)
    if args.out:
        args.out.write_text(rendered)
    return int(not report['passed'])


if __name__ == '__main__':
    raise SystemExit(main())

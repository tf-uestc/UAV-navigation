"""de Boor evaluation of EGO control points and arbitrary knot vectors."""
import bisect
import math
from .contracts import finite, vector3

class BSpline:
    def __init__(self, points, knots, degree):
        self.points = tuple(vector3(p) for p in points)
        self.knots, self.degree = tuple(knots), degree
        if type(degree) is not int or degree < 0 or len(self.points) < degree+1:
            raise ValueError("invalid degree or control count")
        if len(self.knots) != len(self.points)+degree+1:
            raise ValueError("invalid knot count")
        if not all(finite(k) for k in self.knots) or any(a>b for a,b in zip(self.knots,self.knots[1:])):
            raise ValueError("knots must be finite and nondecreasing")
        self.begin, self.end = self.knots[degree], self.knots[len(self.points)]
        if self.end <= self.begin:
            raise ValueError("empty spline domain")
        self.duration = self.end-self.begin
    def evaluate(self, elapsed):
        if not finite(elapsed):
            raise ValueError("nonfinite sample time")
        t = min(self.end,max(self.begin,self.begin+elapsed))
        p = self.degree
        span = len(self.points)-1 if t == self.end else bisect.bisect_right(self.knots,t)-1
        span = min(len(self.points)-1,max(p,span))
        work = [list(self.points[span-p+j]) for j in range(p+1)]
        for r in range(1,p+1):
            for j in range(p,r-1,-1):
                i = span-p+j
                den = self.knots[i+p-r+1]-self.knots[i]
                alpha = (t-self.knots[i])/den if den else 0.0
                work[j] = [(1-alpha)*a+alpha*b for a,b in zip(work[j-1],work[j])]
        return tuple(work[p])
    def derivative(self):
        if self.degree == 0:
            return None
        points = []
        for i in range(len(self.points)-1):
            den = self.knots[i+self.degree+1]-self.knots[i+1]
            factor = self.degree/den if den else 0.0
            points.append(tuple(factor*(b-a) for a,b in zip(self.points[i],self.points[i+1])))
        return BSpline(points,self.knots[1:-1],self.degree-1)
    def samples(self, period=.05):
        if not finite(period) or period <= 0:
            raise ValueError("invalid sample period")
        count = int(math.ceil(self.duration/period))
        if count > 100000:
            raise ValueError("sampling limit exceeded")
        velocity = self.derivative()
        acceleration = velocity.derivative() if velocity else None
        for i in range(count+1):
            t = min(i*period,self.duration)
            yield (t,self.evaluate(t),velocity.evaluate(t) if velocity else (0.,)*3,
                acceleration.evaluate(t) if acceleration else (0.,)*3)

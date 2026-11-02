#include <bspline_opt/uniform_bspline.h>
#include <iomanip>
int main() {
  int degree,n,k,samples;
  if (!(std::cin>>degree>>n>>k>>samples)) return 2;
  Eigen::MatrixXd points(3,n);
  Eigen::VectorXd knots(k);
  for(int i=0;i<n;++i) for(int j=0;j<3;++j) std::cin>>points(j,i);
  for(int i=0;i<k;++i) std::cin>>knots(i);
  ego_planner::UniformBspline b(points,degree,1.0); b.setKnot(knots);
  auto velocity=b.getDerivative(); auto acceleration=velocity.getDerivative();
  double lo,hi; b.getTimeSpan(lo,hi);
  std::cout<<std::setprecision(17);
  for(int i=0;i<samples;++i) {
    double t=(hi-lo)*i/(samples-1);
    auto p=b.evaluateDeBoorT(t),v=velocity.evaluateDeBoorT(t),a=acceleration.evaluateDeBoorT(t);
    std::cout<<t;
    for(int j=0;j<3;++j) std::cout<<" "<<p(j);
    for(int j=0;j<3;++j) std::cout<<" "<<v(j);
    for(int j=0;j<3;++j) std::cout<<" "<<a(j);
    std::cout<<"\n";
  }
}

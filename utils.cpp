#include <Eigen/Geometry>

inline double wrapToPi(double angle){
  double ret=angle;
  while(ret>M_PI){
    ret-=2*M_PI;
  }

  while(ret<=-M_PI){
    ret+=2*M_PI;
  }

  return ret;
}

inline double angDiff(double thetaD, double theta){
  double alpha=0;
  Eigen::Vector2d nD,n;

  nD<<cos(thetaD), sin(thetaD);
  n<<cos(theta), sin(theta);

  double alphaAbs = acos(nD.transpose()*n);

  Eigen::Vector3d n3,nD3;

  n3<<n(0),n(1),0;
  nD3<<nD(0),nD(1),0;

  Eigen::Vector3d nC3;

  nC3=n3.cross(nD3);

  if(nC3(2)>0){
    alpha=alphaAbs;
  }
  else{
    alpha=-alphaAbs;
  }

  return alpha;

}

inline Eigen::MatrixXd matrixPower(Eigen::MatrixXd& A, int exp){

	Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A.rows(),A.cols());

	for (int i=0; i<exp;++i)
        	result *= A;

	return result;
}

inline double sign(double x){
	if(x>0) return +1;
	if(x<0) return -1;
	return -1;
}

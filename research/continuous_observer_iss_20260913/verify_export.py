"""Independently verify the exported continuous ISS matrices with NumPy."""
import itertools,json
from pathlib import Path
import numpy as np
folder=Path(__file__).resolve().parent
saved=json.loads((folder/'certificate_checks.json').read_text())
A=np.zeros((7,7));A[0,1]=A[1,2]=A[3,4]=A[4,5]=1
C=np.eye(7)[[0,3,6]]
G=saved['gnss'];P=np.array(G['P6']);K=np.array(G['K']);N=np.array(G['N']);theta=G['theta']
T=np.diag(np.power(theta,[1,2,3,1,2,3]));Ti=np.linalg.inv(T)
coeff=[(0,1,32),(0,4,32),(1,1,5),(1,4,5),(1,2,16),(1,5,16),(2,1,5),(2,4,5),(2,5,16),(2,2,16)]
worst=-np.inf;count=0
for signs in itertools.product([-1,1],repeat=10):
 H=np.zeros((3,6))
 for (i,j,b),s in zip(coeff,signs):H[i,j]=b*s
 for q,q2 in itertools.product([-.6,.6],[0,.36]):
  M=A[:6,:6].copy();M[2,1]=M[5,4]=q2;M[2,5]=-2*q;M[5,2]=2*q
  F=Ti@M@T-theta*K@C[:2,:6]-theta**-3*N@H@T
  worst=max(worst,np.linalg.eigvalsh(P@F+F.T@P)[-1]);count+=1
assert worst <= -G['analyticUniformMargin']+1e-10
L=saved['lidar'];P,Q,R=[np.array(L[k]) for k in ['P','Q','R']];K=np.array(L['K']);N=np.array(L['N']);d=L['delaySeconds'];rho=L['rate'];g=L['g'];r=np.exp(-2*rho*d)
Ad=-K@C;F=np.column_stack((A,Ad,np.eye(7)))
Pi=np.block([[P@A+A.T@P+2*rho*P+Q-r*R,P@Ad+r*R,P],[(P@Ad+r*R).T,-r*(Q+R),np.zeros((7,7))],[P,np.zeros((7,7)),-g*np.eye(7)]])
block=np.block([[Pi,d*F.T@R],[d*R@F,-R]])
nominal=-np.linalg.eigvalsh(block)[-1]
q=L['maximumCourseRate'];wmin=L['minimumInformationWeight']
delta=q*np.sqrt(q*q+4)+np.linalg.norm(N,2)*np.sqrt(16*16**2+4*5**2+2)+np.linalg.norm(K,2)*(1-wmin)
margin=nominal-2*(np.linalg.norm(P,2)+d*np.linalg.norm(R,2))*delta
assert margin>0 and abs(margin-L['analyticUniformMargin'])<1e-9
# Differentiate the actual integral functional, without using its expanded derivative.
nodes,weights=np.polynomial.legendre.leggauss(64);rng=np.random.default_rng(20260913)
p0,p1,p2=rng.normal(size=(3,7))
def x(t):return p0+t*p1+t*t*p2
def dx(t):return p1+2*t*p2
def integral(f,t):
 u=t-d/2+d*nodes/2
 return d/2*sum(w*f(s) for s,w in zip(u,weights))
def V(t):
 return x(t)@P@x(t)+integral(lambda s: np.exp(2*rho*(s-t))*(x(s)@Q@x(s)),t)+d*integral(lambda s:(s-t+d)*np.exp(2*rho*(s-t))*(dx(s)@R@dx(s)),t)
t=.7;dt=1e-5
lhs=(V(t+dt)-V(t-dt))/(2*dt)+2*rho*V(t)
rhs=2*x(t)@P@dx(t)+2*rho*x(t)@P@x(t)+x(t)@Q@x(t)-r*x(t-d)@Q@x(t-d)+d*d*dx(t)@R@dx(t)-d*integral(lambda s:np.exp(2*rho*(s-t))*(dx(s)@R@dx(s)),t)
relative=abs(lhs-rhs)/max(1,abs(rhs));assert relative<1e-8
result={'numpyVersion':np.__version__,'gnssVerticesChecked':count,'gnssWorstEigenvalue':worst,'lidarRecoveredNominalMargin':nominal,'lidarRecoveredUniformMargin':margin,'functionalDerivativeRelativeResidual':relative,'quadratureOrder':64,'finiteDifferenceStep':dt,'seed':20260913,'passed':True,'scope':'Independent numerical and algebra checks, not interval arithmetic or runtime validation.'}
(folder/'independent_checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

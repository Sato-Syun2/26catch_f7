"""低周波の移動量を抑え、高周波側で速度入力振幅を増やす試験波形。"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class VelocitySweep:
    f0: float = .5
    f1: float = 10.
    seconds: float = 60.
    ramp: float = 5.
    tau: float = .1
    angle: float = 5.
    velocity_cap: float = 600.

    def __post_init__(self):
        if not all(math.isfinite(x) for x in vars(self).values()):
            raise ValueError('nonfinite waveform')
        if not (.5<=self.f0<self.f1<=10 and 10<=self.seconds<=60 and self.ramp==5
                and 0<self.tau<=.5 and 0<self.angle<=5 and 0<self.velocity_cap<=600):
            raise ValueError('invalid waveform bounds')

    @property
    def duration(self): return self.seconds+2*self.ramp

    def sample(self,t):
        if t<=0: return 0.,0.
        k=math.log(self.f1/self.f0)/self.seconds;w0=2*math.pi*self.f0
        s=min(max(t-self.ramp,0.),self.seconds)
        phase=w0*min(t,self.ramp)+w0*math.expm1(k*s)/k
        if t>self.ramp+self.seconds:
            phase+=2*math.pi*self.f1*(t-self.ramp-self.seconds)
        omega=w0*math.exp(k*s)
        z=min(max(min(t,self.duration-t)/self.ramp,0.),1.)
        envelope=6*z**5-15*z**4+10*z**3
        # 正弦定常近似で基準モデルの角振幅を5度以下とする。
        amplitude=min(self.velocity_cap,self.angle*omega*math.hypot(1.,self.tau*omega))
        return envelope*amplitude*math.sin(phase),phase

    def verify(self):
        v=q=0.;lo=hi=peak=0.;dt=.002;beta=math.exp(-dt/self.tau)
        for i in range(int((self.duration+5)/dt)+1):
            u,_=self.sample(i*dt);v=beta*v+(1-beta)*u;q+=v*dt
            lo=min(lo,q);hi=max(hi,q);peak=max(peak,abs(u))
        if max(abs(lo),abs(hi))>8 or peak>600.0001:
            raise ValueError('model excursion/input exceeds bounds')
        return dict(model_position_min=lo,model_position_max=hi,peak_input=peak,model_final_position=q)

if __name__=='__main__':
    for tau in (.1,.2):
        print(tau,VelocitySweep(tau=tau).verify())

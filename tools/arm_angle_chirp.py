"""角度振幅一定の軌道と一次遅れ逆モデル入力。ROSや実機への副作用なし。"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class AngleChirp:
    amplitude: float = 5.
    f0: float = .2
    f1: float = 1.5
    seconds: float = 60.
    ramp: float = 5.
    tau: float = .1

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('nonfinite waveform parameter')
        if not (0 < self.amplitude <= 5 and .1 <= self.f0 < self.f1 <= 2
                and 10 <= self.seconds <= 60 and self.ramp >= 2 and 0 < self.tau <= .5):
            raise ValueError('invalid waveform bounds')

    @property
    def duration(self):
        return self.seconds+2*self.ramp

    def sample(self, t):
        """q[deg], v[deg/s], a[deg/s²], u=v+tau*a, phase[rad]。"""
        if t <= 0 or t >= self.duration:
            return (0.,0.,0.,0.,0.)
        k=math.log(self.f1/self.f0)/self.seconds
        w0=2*math.pi*self.f0
        if t < self.ramp:
            phase=w0*t; omega=w0; omega_dot=0.
        elif t <= self.ramp+self.seconds:
            s=t-self.ramp
            phase=w0*self.ramp+w0*math.expm1(k*s)/k
            omega=w0*math.exp(k*s);omega_dot=k*omega
        else:
            omega=2*math.pi*self.f1;omega_dot=0.
            phase=w0*self.ramp+w0*math.expm1(k*self.seconds)/k+omega*(t-self.ramp-self.seconds)
        if t < self.ramp or t > self.ramp+self.seconds:
            z=t/self.ramp if t < self.ramp else (self.duration-t)/self.ramp
            dz=(1 if t < self.ramp else -1)/self.ramp
            e=6*z**5-15*z**4+10*z**3
            ed=(30*z**4-60*z**3+30*z**2)*dz
            edd=(120*z**3-180*z**2+60*z)*dz**2
        else:
            e=1.;ed=edd=0.
        si=math.sin(phase);co=math.cos(phase);A=self.amplitude
        q=A*e*si
        v=A*(ed*si+e*omega*co)
        acc=A*(edd*si+2*ed*omega*co+e*(omega_dot*co-omega**2*si))
        return q,v,acc,v+self.tau*acc,phase

if __name__=='__main__':
    for wave in (AngleChirp(),AngleChirp(f1=.8,tau=.2)):
        dt=.002; model=position=0.;error=0.;peak=0.
        for i in range(int(wave.duration/dt)+1):
            t=i*dt;q,v,acc,u,_=wave.sample(t)
            if 0<t<wave.duration and min(abs(t-wave.ramp),abs(t-wave.ramp-wave.seconds))>.01:
                h=.00001
                qm,vm,*_=wave.sample(t-h);qp,vp,*_=wave.sample(t+h)
                assert abs((qp-qm)/(2*h)-v)<1e-4
                assert abs((vp-vm)/(2*h)-acc)<1e-3
            beta=math.exp(-dt/wave.tau)
            model=beta*model+(1-beta)*u;position+=model*dt
            error=max(error,abs(model-v));peak=max(peak,abs(u))
            assert abs(q)<=wave.amplitude+1e-9
        assert error<.6 and abs(position)<.03 and peak<180
        print(f'PASS f={wave.f0}..{wave.f1}, peak u={peak:.3f}deg/s, discrete error={error:.4f}deg/s')

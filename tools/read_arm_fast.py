"""Disable後の肘500Hzリング取得と停止振動解析。実機書き込みなし。"""
import argparse,csv,hashlib,json,re,struct,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);out=p.parse_args().directory
elf=Path('build/Debug/26catch_f7.elf');meta=json.loads((out/'metadata.json').read_text())
assert hashlib.sha256(elf.read_bytes()).hexdigest()==meta['local_elf_sha256']
g=subprocess.check_output(['gdb','-batch',str(elf),'-ex','p/x &arm_fast_log','-ex','p sizeof(arm_fast_log)'],text=True)
addr=re.search(r'\$1 = (0x[0-9a-f]+)',g)[1];size=int(re.search(r'\$2 = (\d+)',g)[1]);assert size==65540
cli='/home/taiyo/snap/code/257/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI'
subprocess.run([cli,'-c','port=SWD','mode=HOTPLUG','-u',addr,str(size),str(out/'fast.bin')],check=True,stdout=subprocess.DEVNULL)
b=(out/'fast.bin').read_bytes();count=struct.unpack_from('<I',b)[0]
rows=[struct.unpack_from('<2I6f',b,4+(i%2048)*32) for i in range(max(0,count-2048),count)]
rows=[r for r in rows if meta['measurement_start_f7']*1000<=r[0]<=meta['measurement_end_f7']*1000]
keys=['tick','feedback_tick','position','raw_velocity','filtered_velocity','command_current','reference_current','target']
with (out/'fast.csv').open('w') as f:
 w=csv.writer(f);w.writerow(keys);w.writerows(rows)
if len(rows)<100:raise SystemExit('insufficient samples')
d=np.array(rows);t=(d[:,0]-d[0,0])*.001;dt=np.diff(t);fs=1/np.median(dt)
if np.max(dt)>.003:raise SystemExit('nonuniform sample interval; CSV saved, FFT skipped')
fig,ax=plt.subplots(4,1,figsize=(12,11))
ax[0].plot(t,d[:,2]-d[:,2].mean());ax[0].set_ylabel('Position - mean [deg]')
for j in (3,4):ax[1].plot(t,d[:,j],label=keys[j],lw=.7)
for j in (5,6):ax[2].plot(t,d[:,j],label=keys[j],lw=.7)
ax[1].set_ylabel('Velocity [deg/s]');ax[2].set_ylabel('Current [A / torque-Kt reference]')
freq=np.fft.rfftfreq(len(t),1/fs);window=np.hanning(len(t));spectra={};metrics={}
for j in (2,3,4,5):
 y=d[:,j];y=y-np.polyval(np.polyfit(t,y,1),t)
 spec=abs(np.fft.rfft(y*window))*2/window.sum();spectra[j]=spec
 band=(freq>=.5)&(freq<fs/2);ids=np.flatnonzero(band);peak=ids[np.argmax(spec[band])]
 metrics[keys[j]]=dict(detrended_rms=float(np.sqrt(np.mean(y*y))),raw_rms=float(np.sqrt(np.mean(d[:,j]**2))),mean=float(np.mean(d[:,j])),peak_to_peak=float(np.ptp(d[:,j])),peak_frequency_hz=float(freq[peak]),peak_amplitude=float(spec[peak]))
ax[3].semilogy(freq[1:],spectra[3][1:],label='raw velocity')
ax[3].semilogy(freq[1:],(2*np.pi*freq*spectra[2])[1:],label='position-derived velocity spectrum')
ax[3].set_xlabel('Frequency [Hz]');ax[3].set_ylabel('Amplitude [deg/s]');ax[3].set_xlim(.5,fs/2)
for x in ax[:3]:x.set_xlabel('Time [s]')
for x in ax:x.grid()
for x in ax[1:]:x.legend()
fig.tight_layout();fig.savefig(out/'fast.png',dpi=150)
metrics.update(samples=len(t),sample_hz=float(fs),max_gap_ms=float(dt.max()*1000),max_feedback_age_ms=float(np.max(d[:,0]-d[:,1])),duplicate_feedback_ticks=int(np.count_nonzero(np.diff(d[:,1])==0)),target_peak=float(np.max(abs(d[:,7]))),saturation_fraction=float(np.mean(abs(d[:,5])>=10.78)))
(out/'fast_summary.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))

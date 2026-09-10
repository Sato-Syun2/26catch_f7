"""リンカ配置検証。Debug/Release ELFの通常RAMと専用LwIPヒープの分離を確認。"""
import argparse,re,subprocess
p=argparse.ArgumentParser();p.add_argument('elf',nargs='+');args=p.parse_args()
for elf in args.elf:
    names=['__lwip_heap_start__','__lwip_heap_end__','_ebss','_end','_sstack','_estack']
    cmd=['gdb','-batch',elf]
    for name in names:cmd+=['-ex','p/x &'+name]
    values=[int(x,16) for x in re.findall(r'\$\d+ = (0x[0-9a-f]+)',subprocess.check_output(cmd,text=True))]
    assert len(values)==len(names),'missing linker symbols'
    lo,hi,bss,end,stack,top=values
    assert 0x20000000<bss<=end<stack<top<=lo<hi<=0x20080000
    assert hi-lo>=8192
    sections=subprocess.check_output(['readelf','-SW',elf],text=True)
    found=False
    for name,addr,size in re.findall(r'\[\s*\d+\]\s+(\S+)\s+\S+\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)',sections):
        addr=int(addr,16);size=int(size,16)
        if name=='.lwip_heap':
            assert addr==lo and size==hi-lo;found=True
        elif size and addr<hi and addr+size>lo:raise AssertionError('overlap: '+name)
    assert found
    print(elf,'PASS',dict(zip(names,map(hex,values))))

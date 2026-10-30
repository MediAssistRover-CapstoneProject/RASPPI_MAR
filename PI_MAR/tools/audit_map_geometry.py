"""Read-only metric map audit. Estimates are not a calibration or automatic resize."""
import argparse,json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pi'))
from mediassist.map_tools import load_trinary_grid,map_fingerprint

def quantile(values,q):
    values=sorted(values);f=(len(values)-1)*q;i=int(f)
    return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(f-i)

def estimate(grid):
    w=grid['width'];r=grid['resolution'];occupied=[(i%w,i//w) for i,v in enumerate(grid['data']) if v>=65]
    free=[(i%w,i//w) for i,v in enumerate(grid['data']) if v==0]
    if len(occupied)<20 or len(free)<20:raise ValueError('INSUFFICIENT_MAP_GEOMETRY')
    # Bound diagnostic work for large maps; never modify the grid being audited.
    occupied=occupied[::max(1,len(occupied)//5000)];free=free[::max(1,len(free)//100000)]
    best=None
    for k in range(900):
        a=k*math.pi/1800;c,s=math.cos(a),math.sin(a)
        u=[x*c+y*s for x,y in occupied];v=[-x*s+y*c for x,y in occupied]
        size=[quantile(u,.99)-quantile(u,.01),quantile(v,.99)-quantile(v,.01)]
        if best is None or size[0]*size[1]<best[0]:best=(size[0]*size[1],a,size)
    _,a,size=best;c,s=math.cos(a),math.sin(a);spans=[]
    for values in ([x*c+y*s for x,y in free],[-x*s+y*c for x,y in free]):
        bins={}
        for v in values:
            i=math.floor(v*2);bins[i]=bins.get(i,0)+1
        threshold=max(bins.values())*.4;dense=[i for i,n in bins.items() if n>threshold]
        spans.append((max(dense)-min(dense)+1)*.5*r)
    return dict(wall_axis_deg=round(math.degrees(a),2),
        wall_bounds_trimmed_1pct_m=sorted(round(v*r,4) for v in size),
        dense_free_span_estimate_m=sorted(round(v,4) for v in spans),
        method='Rotated wall bounds plus 40%-peak free-cell projection. Rectangular-layout estimate only; doorways, objects and noise affect this estimate.')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map-yaml',type=Path,required=True)
    p.add_argument('--expected-width',type=float);p.add_argument('--expected-length',type=float)
    a=p.parse_args();g=load_trinary_grid(a.map_yaml);out=dict(map_yaml=str(a.map_yaml),asset_hash=map_fingerprint(a.map_yaml),
        width_px=g['width'],height_px=g['height'],resolution_m_per_pixel=g['resolution'],origin=g['origin'],
        canvas_m=[g['width']*g['resolution'],g['height']*g['resolution']],**estimate(g))
    if a.expected_width is not None and a.expected_length is not None:
        expected=sorted([a.expected_width,a.expected_length])
        if not all(math.isfinite(v) and v>0 for v in expected):p.error('Expected dimensions must be positive metres')
        out['expected_layout_m']=expected
        out['estimated_difference_m']=[round(v-e,4) for v,e in zip(out['dense_free_span_estimate_m'],expected)]
    print(json.dumps(out,indent=2))
if __name__=='__main__':main()

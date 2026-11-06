"""Pixel-only circular gauge baseline. No simulator, USD, site or truth imports."""
from dataclasses import dataclass, asdict
import math
import json
from pathlib import Path
import cv2
import numpy as np


@dataclass(frozen=True)
class Calibration:
    vmin: float
    vmax: float
    unit: str
    start_angle_deg: float = 225.0
    direction: str = 'clockwise'
    sweep_deg: float = 270.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.vmin,self.vmax,self.start_angle_deg,self.sweep_deg)):
            raise ValueError('nonfinite_calibration')
        if self.vmax<=self.vmin or not 0<self.sweep_deg<360 or self.direction not in ('clockwise','counterclockwise'):
            raise ValueError('invalid_calibration')


def angle_to_value(angle, calibration):
    c=calibration if isinstance(calibration,Calibration) else Calibration(**calibration)
    delta=((c.start_angle_deg-angle) if c.direction=='clockwise' else (angle-c.start_angle_deg))%360
    if delta>c.sweep_deg:
        if 360-delta<=2: delta=0
        elif delta-c.sweep_deg<=2: delta=c.sweep_deg
        else: return None
    return c.vmin+(c.vmax-c.vmin)*delta/c.sweep_deg


def default_settings():
    return json.loads(Path(__file__).with_name('settings.json').read_text(encoding='utf-8'))


def failure(gauge_id,unit,status,error,bbox=None,confidence=0.0,**extra):
    return dict(gauge_id=gauge_id,value=None,unit=unit,bbox=bbox,confidence=float(confidence),
                read_status=status,error=error,**extra)


def _read_circle(gray,circle,cfg):
    cx,cy,r=map(float,circle)
    h,w=gray.shape
    yy,xx=np.ogrid[:h,:w]
    rr=(xx-cx)**2+(yy-cy)**2
    disk=rr<(r*.88)**2
    pixels=gray[disk]
    if pixels.size<100: return None
    white=float(np.percentile(pixels,85)); dark=float(np.percentile(pixels,1))
    if white-dark<35: return None
    rim_angles=np.linspace(0,2*math.pi,120,endpoint=False)
    def ring(radius):
        xs=np.clip(np.rint(cx+radius*np.cos(rim_angles)).astype(int),0,w-1)
        ys=np.clip(np.rint(cy-radius*np.sin(rim_angles)).astype(int),0,h-1)
        return gray[ys,xs].astype(float)
    inner=ring(r*.76)
    outer=np.minimum.reduce([ring(r*f) for f in (.98,1.02,1.06)])
    rim_support=float(np.mean(inner-outer>25))
    if rim_support<.55: return None
    threshold=white-max(35,0.35*(white-dark))
    white_fraction=float(np.mean(pixels>threshold))
    if white_fraction<cfg['min_white_fraction']: return None
    # Refine the hub using only central dark pixels, independent of needle angle.
    centre=(rr<(r*.12)**2)&(gray<threshold)
    if centre.sum()>=3:
        ys,xs=np.nonzero(centre)
        rx,ry=float(xs.mean()),float(ys.mean())
        if math.hypot(rx-cx,ry-cy)<r*.10: cx,cy=rx,ry
    angles=np.arange(0,360,cfg['angle_step_deg'])
    theta=np.deg2rad(angles)
    radii=np.linspace(.22*r,.79*r,max(25,int(r)))
    # Average a narrow angular wedge; long needle wins over the short tail/text.
    responses=[]
    for offset in (-0.7,0,0.7):
        t=theta+math.radians(offset)
        xs=np.clip(np.rint(cx+np.cos(t[:,None])*radii).astype(int),0,w-1)
        ys=np.clip(np.rint(cy-np.sin(t[:,None])*radii).astype(int),0,h-1)
        responses.append(gray[ys,xs].astype(float))
    samples=np.minimum.reduce(responses)
    coverage=np.mean(samples<threshold,axis=1)
    darkness=np.mean(np.clip((white-samples)/max(white-dark,1),0,1),axis=1)
    score=.7*coverage+.3*darkness
    winner=int(np.argmax(score)); angle=float(angles[winner])
    separation=np.abs((angles-angle+180)%360-180)
    runner_up=float(np.max(score[separation>12]))
    margin=float(score[winner]-runner_up)
    confidence=float(np.clip(.75*coverage[winner]+.25*min(1,margin/.35),0,1))
    # Demand support close to the hub and over most of the useful radial span.
    inner=float(np.mean(samples[winner,:max(5,len(radii)//3)]<threshold))
    if coverage[winner]<cfg['min_ray_coverage'] or margin<cfg['min_peak_margin'] or inner<.6:
        return dict(cx=cx,cy=cy,r=r,confidence=confidence,error='ambiguous_or_occluded_needle',
                    coverage=float(coverage[winner]),peak_margin=margin)
    return dict(cx=cx,cy=cy,r=r,angle_deg=angle,confidence=confidence,error=None,
                coverage=float(coverage[winner]),peak_margin=margin)


def read_gauge(rgb,calibration,gauge_id='',roi=None,settings=None):
    c=calibration if isinstance(calibration,Calibration) else Calibration(**calibration)
    cfg=default_settings() if settings is None else dict(settings)
    if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or min(rgb.shape[:2])<16:
        return failure(gauge_id,c.unit,'unreadable','invalid_rgb_uint8')
    h,w=rgb.shape[:2]; ox=oy=0; crop=rgb
    if roi is not None:
        if len(roi)!=4 or any(not math.isfinite(float(v)) for v in roi):
            return failure(gauge_id,c.unit,'unreadable','invalid_roi')
        x,y,rw,rh=map(int,roi)
        if rw<16 or rh<16 or x<0 or y<0 or x+rw>w or y+rh>h:
            return failure(gauge_id,c.unit,'unreadable','invalid_roi')
        ox,oy=x,y;crop=rgb[y:y+rh,x:x+rw]
    gray=cv2.cvtColor(crop,cv2.COLOR_RGB2GRAY)
    # Bound detector cost on 720p; all measurements below remain in native pixels.
    scale=min(1.,960/max(gray.shape))
    small=cv2.resize(gray,None,fx=scale,fy=scale) if scale<1 else gray
    blurred=cv2.GaussianBlur(small,(5,5),1)
    circles=cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,1,24*scale,
             param1=100,param2=cfg['hough_param2'],minRadius=max(8,int(cfg['min_radius_px']*scale)),
             maxRadius=int(min(small.shape)*.46))
    if circles is None: return failure(gauge_id,c.unit,'not_found','dial_not_found')
    candidates=[]
    for circ in circles[0][:100]:
        circle=np.asarray(circ)/scale
        cx,cy,r=circle
        if cx-r<0 or cy-r<0 or cx+r>=gray.shape[1] or cy+r>=gray.shape[0]: continue
        result=_read_circle(gray,circle,cfg)
        if result is not None: candidates.append(result)
    if not candidates: return failure(gauge_id,c.unit,'not_found','no_plausible_dial')
    candidates.sort(key=lambda d:(d['error'] is None,d['confidence']),reverse=True)
    best=candidates[0]
    x,y,r=best['cx']+ox,best['cy']+oy,best['r']
    bbox=[max(0,int(x-r)),max(0,int(y-r)),min(w-int(x-r),int(2*r)+1),min(h-int(y-r),int(2*r)+1)]
    diagnostics={**best,'cx':x,'cy':y}
    if best['error'] or best['confidence']<cfg['min_confidence']:
        return failure(gauge_id,c.unit,'unreadable',best['error'] or 'low_confidence',bbox,best['confidence'],diagnostics=diagnostics)
    others=[d for d in candidates[1:] if d['error'] is None and math.hypot(d['cx']-best['cx'],d['cy']-best['cy'])>best['r']]
    if others and others[0]['confidence']>=best['confidence']-.1:
        return failure(gauge_id,c.unit,'unreadable','multiple_plausible_dials',bbox,best['confidence'])
    value=angle_to_value(best['angle_deg'],c)
    if value is None:
        return failure(gauge_id,c.unit,'unreadable','needle_outside_scale',bbox,best['confidence'],diagnostics=diagnostics)
    return dict(gauge_id=gauge_id,value=float(value),unit=c.unit,bbox=bbox,confidence=best['confidence'],
                read_status='ok',error=None,diagnostics=diagnostics)


def annotate(rgb,result):
    out=rgb.copy()
    colour=(20,200,80) if result['read_status']=='ok' else (240,70,50)
    if result.get('bbox'):
        x,y,w,h=result['bbox'];cv2.rectangle(out,(x,y),(x+w,y+h),colour,2)
    d=result.get('diagnostics',{})
    if 'angle_deg' in d:
        x,y=int(d['cx']),int(d['cy']);a=math.radians(d['angle_deg'])
        tip=(int(x+.8*d['r']*math.cos(a)),int(y-.8*d['r']*math.sin(a)))
        cv2.arrowedLine(out,(x,y),tip,(0,120,255),2,tipLength=.15)
    label=f"{result['gauge_id']}: {result['value'] if result['value'] is not None else result['error']} {result['unit']}"
    cv2.putText(out,label,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.65,colour,2)
    return out

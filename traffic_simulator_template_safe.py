from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import math, time

WINDOW_W = 1200
WINDOW_H = 800

fovY          = 68.0
cam_angle     = 44.0
cam_height    = 520.0
cam_radius    = 820.0
cam_mode      = "TOP"
follow_idx    = 0
smooth_eye    = None
smooth_look   = None
camera_pos    = (0.0, 820.0, 520.0)

LANE_W        = 38.0
ROAD_HALF     = 90.0
SPAWN_DIST    = 650.0
INTER_HALF    = 110.0
STOP_LINE     = 135.0
PED_STOP_EXTRA= 52.0
SAFE_GAP      = 88.0
GRID_LEN      = 700.0
MAX_VEH       = 24

DIR_NAMES   = ["EAST","NORTH","WEST","SOUTH"]
DIR_VEC     = [(1,0),(0,1),(-1,0),(0,-1)]
AUTO_ORDER  = [1,0,3,2]
BTN_DIRS    = [("N",1),("E",0),("S",3),("W",2)]

vehicles             = []
wrecks               = []
completed_count      = 0
total_wait           = 0.0
sim_time             = 0.0
last_wall            = None
paused               = False
spawn_on             = True
spawn_timer          = 0.0
SPAWN_INTERVAL       = 1.85
crash_count          = 0

traffic_auto   = True
dir_state      = ["RED","GREEN","RED","RED"]
dir_timer      = [0.0, 0.0, 0.0, 0.0]
dir_prev_state = ["RED","GREEN","RED","RED"]
auto_green_dir = 1
auto_phase     = "GREEN"
auto_timer     = 0.0
GREEN_DUR      = 9.0
YELLOW_DUR     = 2.2

CONFLICT_PAIRS = {(0,2),(2,0),(1,3),(3,1),(0,1),(1,0),(0,3),(3,0),
                  (1,2),(2,1),(2,3),(3,2)}

ped_active     = False
block_active   = False
manual_incident= False
BLOCK_DIR      = 0
BLOCK_LANE     = 1
BLOCK_LONG     = -305.0

day_mode    = True
day_night_t = 0.0

queue_counts = [0,0,0,0]
BUTTONS      = []

_seed = 423
def _rng():
    global _seed
    _seed = (_seed*1103515245+12345)&0x7FFFFFFF
    return _seed
def rand01():   return (_rng()%10000)/10000.0
def randi(n):   return _rng()%n

def clamp(v,lo,hi): return lo if v<lo else hi if v>hi else v
def lerp(a,b,t):    return a+(b-a)*clamp(t,0,1)
def lerp3(a,b,t):   return tuple(a[i]+(b[i]-a[i])*clamp(t,0,1) for i in range(3))
def scene_col(d,n): return lerp3(d,n,day_night_t)

def _col(c): glColor3f(c[0],c[1],c[2])

def box_world(cx,cy,zb,sx,sy,sz,col):
    glPushMatrix(); _col(col)
    glTranslatef(cx,cy,zb+sz*0.5)
    glScalef(sx,sy,sz); glutSolidCube(1); glPopMatrix()

def box_local(lx,ly,lz,sx,sy,sz,col):
    glPushMatrix(); _col(col)
    glTranslatef(lx,ly,lz)
    glScalef(sx,sy,sz); glutSolidCube(1); glPopMatrix()

def slab(x0,y0,x1,y1,z,col,th=0.9):
    box_world((x0+x1)*.5,(y0+y1)*.5,z,
              max(0.4,abs(x1-x0)),max(0.4,abs(y1-y0)),th,col)

def cyl_z(cx,cy,z,r,h,col):
    glPushMatrix(); _col(col)
    glTranslatef(cx,cy,z)
    gluCylinder(gluNewQuadric(),r,r,h,12,5); glPopMatrix()

def cone_z(cx,cy,z,rb,rt,h,col):
    glPushMatrix(); _col(col)
    glTranslatef(cx,cy,z)
    gluCylinder(gluNewQuadric(),rb,rt,h,12,5); glPopMatrix()

def sphere(cx,cy,cz,r,col):
    glPushMatrix(); _col(col)
    glTranslatef(cx,cy,cz)
    gluSphere(gluNewQuadric(),r,12,10); glPopMatrix()

def enter_2d():
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    gluOrtho2D(0,WINDOW_W,0,WINDOW_H)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()

def exit_2d():
    glPopMatrix()
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

def hud_box(x0,y0,x1,y1,col,alpha=0.55):
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    glColor4f(col[0],col[1],col[2],alpha)
    glBegin(GL_QUADS)
    glVertex2f(x0,y0); glVertex2f(x1,y0)
    glVertex2f(x1,y1); glVertex2f(x0,y1)
    glEnd()
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)

def hud_rounded(x0,y0,x1,y1,col,alpha=0.72,r=5):
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    glColor4f(col[0],col[1],col[2],alpha)
    glBegin(GL_QUADS)
    glVertex2f(x0+r,y0); glVertex2f(x1-r,y0)
    glVertex2f(x1-r,y1); glVertex2f(x0+r,y1)
    glVertex2f(x0,y0+r); glVertex2f(x1,y0+r)
    glVertex2f(x1,y1-r); glVertex2f(x0,y1-r)
    glEnd()
    steps=8
    for cx2,cy2,a0,a1 in [(x0+r,y0+r,180,270),(x1-r,y0+r,270,360),
                           (x1-r,y1-r,0,90),  (x0+r,y1-r,90,180)]:
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(cx2,cy2)
        for si in range(steps+1):
            ang=math.radians(a0+(a1-a0)*si/steps)
            glVertex2f(cx2+math.cos(ang)*r,cy2+math.sin(ang)*r)
        glEnd()
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)

def hud_border(x0,y0,x1,y1,col,alpha=0.9,lw=1.5):
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    glColor4f(col[0],col[1],col[2],alpha)
    glLineWidth(lw)
    glBegin(GL_LINE_LOOP)
    glVertex2f(x0,y0); glVertex2f(x1,y0)
    glVertex2f(x1,y1); glVertex2f(x0,y1)
    glEnd()
    glLineWidth(1.0)
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)

def draw_text(x,y,txt,font=GLUT_BITMAP_HELVETICA_12,col=(1,1,1)):
    _col(col)
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    gluOrtho2D(0,WINDOW_W,0,WINDOW_H)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glRasterPos2f(x,y)
    for ch in txt: glutBitmapCharacter(font,ord(ch))
    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

def right_vec(d):
    dx,dy=DIR_VEC[d]; return (dy,-dx)

def lane_off(li):
    return LANE_W*0.5+li*LANE_W

def lane_pt(d,li,s):
    dx,dy=DIR_VEC[d]; rx,ry=right_vec(d); o=lane_off(li)
    return (dx*s+rx*o, dy*s+ry*o)

def bezier2(p0,p1,p2,t):
    a=(1-t)**2; b=2*(1-t)*t; c=t*t
    return (a*p0[0]+b*p1[0]+c*p2[0], a*p0[1]+b*p1[1]+c*p2[1])

def exit_dir(start,turn):
    if turn=="left":  return (start+1)%4
    if turn=="right": return (start+3)%4
    return start

def build_path(start,turn,lane):
    pts=[lane_pt(start,lane,-SPAWN_DIST)]
    if turn=="straight":
        pts+=[lane_pt(start,lane,-INTER_HALF),
              lane_pt(start,lane, INTER_HALF),
              lane_pt(start,lane, SPAWN_DIST)]
    else:
        ed=exit_dir(start,turn)
        entry=lane_pt(start,lane,-INTER_HALF)
        ex_pt=lane_pt(ed,lane,INTER_HALF)
        irx,iry=right_vec(start); orx,ory=right_vec(ed); o=lane_off(lane)
        ctrl=(irx*o+orx*o, iry*o+ory*o)
        pts.append(entry)
        for i in range(1,32): pts.append(bezier2(entry,ctrl,ex_pt,i/31.0))
        pts+=[lane_pt(ed,lane,INTER_HALF), lane_pt(ed,lane,SPAWN_DIST)]

    cl=[pts[0]]
    for p in pts[1:]:
        if (p[0]-cl[-1][0])**2+(p[1]-cl[-1][1])**2>1e-4: cl.append(p)
    cum=[0.0]
    for i in range(1,len(cl)):
        dx=cl[i][0]-cl[i-1][0]; dy=cl[i][1]-cl[i-1][1]
        cum.append(cum[-1]+math.sqrt(dx*dx+dy*dy))
    return {"pts":cl,"cum":cum,"len":cum[-1]}

def sample_path(path,s):
    pts=path["pts"]; cum=path["cum"]
    if s<=0: i=0
    elif s>=path["len"]: i=len(pts)-2
    else:
        i=0
        while i<len(cum)-2 and cum[i+1]<s: i+=1
    seg=cum[i+1]-cum[i]
    t=clamp((s-cum[i])/seg,0,1) if seg>1e-4 else 0.0
    x=pts[i][0]+(pts[i+1][0]-pts[i][0])*t
    y=pts[i][1]+(pts[i+1][1]-pts[i][1])*t
    h=math.atan2(pts[i+1][1]-pts[i][1],pts[i+1][0]-pts[i][0])
    return x,y,h

def adiff(a,b):
    d=a-b
    while d>180:  d-=360
    while d<-180: d+=360
    return d

PALETTE=[(0.88,0.08,0.08),(0.06,0.34,0.96),(0.94,0.72,0.08),
         (0.10,0.72,0.28),(0.76,0.18,0.88),(0.92,0.92,0.92),
         (0.08,0.76,0.78),(0.96,0.44,0.10),(0.20,0.20,0.20),
         (0.94,0.50,0.60),(0.22,0.80,0.46),(0.70,0.30,0.10)]

class Vehicle:
    def __init__(self,start,turn,lane,init_s=0.0):
        self.start=start; self.turn=turn; self.lane=lane
        self.exit=exit_dir(start,turn)
        self.path=build_path(start,turn,lane)
        self.dist=init_s
        self.stop_s=SPAWN_DIST-STOP_LINE

        self.kind="bus" if rand01()<0.13 else "car"
        if self.kind=="bus":
            self.L=84.0;self.W=32.0;self.H=28.0
            self.max_v=50+rand01()*10;self.accel=26.0;self.brake=70.0
        else:
            self.L=54.0;self.W=28.0;self.H=18.0
            self.max_v=60+rand01()*20;self.accel=40+rand01()*8;self.brake=90.0

        self.speed=8+rand01()*16
        self.drv_f=0.90+rand01()*0.20
        self.color=(0.95,0.78,0.18) if self.kind=="bus" else PALETTE[randi(len(PALETTE))]
        self.wait_t=0.0
        self.braking=False
        self.stopped=False
        self.steer=0.0
        self.wroll=0.0
        self.idle_ph=rand01()*6.28
        self.x=self.y=0.0
        self.heading=0.0
        self.hdeg=0.0
        self.completed=False
        self.crashed=False
        self.crash_timer=0.0
        self.shake_t=0.0
        self.shake_amp=0.0
        self.push_x=self.push_y=0.0
        self.is_wreck=False
        self.removed=False
        self.rev_timer=0.0
        _update_pose(self)

def _update_pose(v):
    x,y,h=sample_path(v.path,v.dist)
    x2,y2,h2=sample_path(v.path,min(v.path["len"],v.dist+22))
    v.x=x;v.y=y;v.heading=h;v.hdeg=math.degrees(h)
    v.steer=clamp(adiff(math.degrees(h2),math.degrees(h))*2.2,-30,30)

def sig_color(direction):
    return dir_state[direction]

def desired_speed(v):
    if v.is_wreck: return 0.0, v.dist
    if v.crash_timer>0: return 0.0, None

    target=v.max_v*v.drv_f
    stop_at=None
    front_s=v.dist+v.L*0.48
    d2stop=v.stop_s-front_s
    sig=sig_color(v.start)

    ped_stop_s=v.stop_s+PED_STOP_EXTRA
    d2ped=ped_stop_s-front_s

    if ped_active and -5<d2ped<280:
        target=min(target,max(0,d2ped*1.05))
        stop_at=ped_stop_s

    if -5<d2stop<250:
        must=False
        if sig=="RED": must=True
        elif sig=="YELLOW":
            if d2stop>42: must=True
            else: target=min(target,max(0,d2stop*1.05))
        if must:
            target=min(target,max(0,d2stop*1.10))
            if stop_at is None or v.stop_s<stop_at:
                stop_at=v.stop_s

    if block_active and v.start==BLOCK_DIR and v.lane==BLOCK_LANE:
        bs=SPAWN_DIST+BLOCK_LONG-42
        d2b=bs-front_s
        if -4<d2b<380:
            target=min(target,max(0,d2b*1.05))
            if stop_at is None or bs<stop_at: stop_at=bs

    for w in wrecks:
        if w.start==v.start and w.lane==v.lane:
            gap=w.dist-v.dist-(v.L+w.L)*0.52
            if 0<gap<380:
                target=min(target,max(0,gap*1.05))
                ws=v.dist+max(0,gap-6)
                if stop_at is None or ws<stop_at: stop_at=ws

    for o in vehicles:
        if o is v or o.is_wreck: continue
        if (o.start==v.start and o.lane==v.lane
                and o.dist>v.dist
                and o.dist<v.stop_s+280 and v.dist<v.stop_s+280):
            gap=o.dist-v.dist-(v.L+o.L)*0.52
            desired_g=SAFE_GAP+v.speed*0.24
            if gap<desired_g:
                target=min(target,max(0,gap*1.08))
                if gap<10:
                    ws=v.dist+max(0,gap-6)
                    if stop_at is None or ws<stop_at: stop_at=ws

        dx=o.x-v.x; dy=o.y-v.y
        fwd=math.cos(v.heading)*dx+math.sin(v.heading)*dy
        lat=abs(-math.sin(v.heading)*dx+math.cos(v.heading)*dy)
        near=abs(v.x)<165 and abs(v.y)<165 and abs(o.x)<165 and abs(o.y)<165
        if near and 0<fwd<80 and lat<36:
            target=min(target,max(0,(fwd-36)*1.10))

    return clamp(target,0,v.max_v),stop_at

def add_vehicle(start,turn,lane,init_s=0.0):
    if len(vehicles)>=MAX_VEH: return False
    nv=Vehicle(start,turn,lane,init_s)
    for o in vehicles:
        if o.start==start and o.lane==lane and abs(o.dist-nv.dist)<120: return False
    vehicles.append(nv); return True

def choose_tl():
    r=randi(10)
    if r<2: return "left",0
    if r<7: return "straight",0 if randi(3) else 1
    return "right",1

def try_spawn():
    d=randi(4); turn,lane=choose_tl()
    for v in vehicles:
        if v.start==d and v.lane==lane and v.dist<155: return False
    return add_vehicle(d,turn,lane,0.0)

def collision_step():
    global crash_count
    for i in range(len(vehicles)):
        a=vehicles[i]
        if a.is_wreck or a.crash_timer>0: continue
        for j in range(i+1,len(vehicles)):
            b=vehicles[j]
            if b.is_wreck or b.crash_timer>0: continue
            dx=a.x-b.x; dy=a.y-b.y; d2=dx*dx+dy*dy
            md=(a.W+b.W)*0.55
            if d2<md*md and d2>0.01:
                conflict=(not traffic_auto) and ((a.start,b.start) in CONFLICT_PAIRS)
                both_in=(abs(a.x)<INTER_HALF+20 and abs(a.y)<INTER_HALF+20 and
                         abs(b.x)<INTER_HALF+20 and abs(b.y)<INTER_HALF+20)
                if conflict or both_in:
                    if a.shake_t<=0 and b.shake_t<=0:
                        d=math.sqrt(d2)+0.001
                        nx=dx/d; ny=dy/d
                        amp=clamp((a.speed+b.speed)*0.14,3.0,12.0)
                        for vv,sx,sy in [(a,nx,ny),(b,-nx,-ny)]:
                            vv.shake_amp=amp; vv.shake_t=2.0
                            vv.crash_timer=4.0
                            vv.speed=0; vv.crashed=True
                            vv.push_x+=sx*8; vv.push_y+=sy*8
                        crash_count+=1

def promote_to_wreck(v):
    v.is_wreck=True
    v.speed=0; v.crash_timer=0
    wrecks.append(v)

def set_direction_green(d):
    global dir_prev_state
    for i in range(4):
        dir_prev_state[i]=dir_state[i]
        if i==d:
            dir_state[i]="GREEN"; dir_timer[i]=0.0
        else:
            if dir_state[i]=="GREEN":
                dir_state[i]="YELLOW"; dir_timer[i]=0.0
            elif dir_state[i]!="YELLOW":
                dir_state[i]="RED";   dir_timer[i]=0.0

def request_manual_green(d):
    global traffic_auto
    traffic_auto=False
    set_direction_green(d)

def toggle_auto():
    global traffic_auto,auto_green_dir,auto_phase,auto_timer
    traffic_auto=not traffic_auto
    if traffic_auto:
        auto_green_dir=1; auto_phase="GREEN"; auto_timer=0.0
        set_direction_green(auto_green_dir)

def adaptive_green():
    q=queue_counts[auto_green_dir]
    return GREEN_DUR+clamp(q*0.8,0,7.0)

def update_signal(dt):
    global auto_green_dir,auto_phase,auto_timer,dir_prev_state

    for i in range(4):
        if dir_state[i]=="YELLOW":
            dir_timer[i]+=dt
            if dir_timer[i]>=YELLOW_DUR:
                dir_prev_state[i]="YELLOW"
                dir_state[i]="RED"; dir_timer[i]=0.0

    if traffic_auto:
        auto_timer+=dt
        if auto_phase=="GREEN" and auto_timer>=adaptive_green():
            auto_phase="YELLOW"; auto_timer=0.0
            dir_state[auto_green_dir]="YELLOW"; dir_timer[auto_green_dir]=0.0
        elif auto_phase=="YELLOW" and auto_timer>=YELLOW_DUR:
            auto_phase="GREEN"; auto_timer=0.0
            next_d=AUTO_ORDER[(AUTO_ORDER.index(auto_green_dir)+1)%4]
            auto_green_dir=next_d
            set_direction_green(auto_green_dir)

    for v in vehicles:
        if v.is_wreck: continue
        was_stopped=(v.speed<4 and v.stopped)
        now_green=(dir_state[v.start]=="GREEN")
        prev_was_not=(dir_prev_state[v.start] in ("RED","YELLOW"))
        if was_stopped and now_green and prev_was_not:
            v.rev_timer=1.6

def sig_remaining():
    if traffic_auto:
        if auto_phase=="GREEN": return max(0,adaptive_green()-auto_timer)
        return max(0,YELLOW_DUR-auto_timer)
    return 0.0

def count_queues():
    global queue_counts
    queue_counts=[0,0,0,0]
    for v in vehicles:
        if v.is_wreck: continue
        d2s=v.stop_s-(v.dist+v.L*0.48)
        if -8<d2s<260 and v.speed<10: queue_counts[v.start]+=1

def sig_bulbs(d):
    s=dir_state[d]; DIM=0.10
    if s=="RED":    return (1.0,0.05,0.03),(DIM,DIM*.8,.01),(0.01,DIM,0.02)
    if s=="YELLOW": return (0.20,.02,.02),(1.0,0.86,0.08),(0.01,DIM,0.02)
    return (0.20,.02,.02),(DIM,DIM*.8,.01),(0.05,1.0,0.10)

def draw_signal_head(d,bx,by):
    dx,dy=DIR_VEC[d]
    red,yel,grn=sig_bulbs(d)
    hc=(0.02,0.02,0.02); hood=(0.05,0.05,0.05)
    if abs(dx)>0.5:
        box_world(bx,by,78,16,34,58,hc)
        box_world(bx+dx*3,by,75,4,40,64,(0.01,0.01,0.01))
        bx2=bx-dx*9; hx=bx-dx*13
        for pz,col in [(126,red),(109,yel),(92,grn)]:
            box_world(hx,by,pz-2,9,24,4,hood)
            box_world(hx,by-11,pz-7,9,3,14,hood)
            box_world(hx,by+11,pz-7,9,3,14,hood)
            sphere(bx2,by,pz,7.0,col)
    else:
        box_world(bx,by,78,34,16,58,hc)
        box_world(bx,by+dy*3,75,40,4,64,(0.01,0.01,0.01))
        by2=by-dy*9; hy=by-dy*13
        for pz,col in [(126,red),(109,yel),(92,grn)]:
            box_world(bx,hy,pz-2,24,9,4,hood)
            box_world(bx-11,hy,pz-7,3,9,14,hood)
            box_world(bx+11,hy,pz-7,3,9,14,hood)
            sphere(bx,by2,pz,7.0,col)

def draw_traffic_light(d):
    dx,dy=DIR_VEC[d]; rx,ry=right_vec(d)
    bx=dx*(-STOP_LINE-6)+rx*LANE_W
    by=dy*(-STOP_LINE-6)+ry*LANE_W
    px=dx*(-STOP_LINE-38)+rx*(ROAD_HALF+66)
    py=dy*(-STOP_LINE-38)+ry*(ROAD_HALF+66)
    pc=scene_col((0.22,0.22,0.22),(0.10,0.10,0.12))
    bc=scene_col((0.40,0.38,0.36),(0.16,0.16,0.18))
    box_world(px,py,0,24,24,10,bc)
    cyl_z(px,py,9,4.2,118,pc)
    z=118
    jx=bx; jy=py
    sx2=max(6,abs(jx-px)); sy2=max(6,abs(jy-py))
    box_world((px+jx)*.5,(py+jy)*.5,z,sx2,sy2,5,pc)
    sx3=max(6,abs(bx-jx)); sy3=max(6,abs(by-jy))
    box_world((jx+bx)*.5,(jy+by)*.5,z,sx3,sy3,5,pc)
    cyl_z(bx,by,108,2.5,18,pc)
    box_world(bx,by,104,10,10,8,pc)
    draw_signal_head(d,bx,by)

def draw_wheel(lx,ly,v,front):
    glPushMatrix()
    glTranslatef(lx,ly,7.5)
    if front: glRotatef(v.steer,0,0,1)
    glPushMatrix(); glColor3f(0.06,0.06,0.06)
    glRotatef(90,1,0,0); glTranslatef(0,0,-3.2)
    gluCylinder(gluNewQuadric(),7.0,7.0,6.4,12,4); glPopMatrix()
    glPushMatrix(); glColor3f(0.72,0.72,0.74)
    glRotatef(v.wroll,1,0,0)
    glRotatef(90,1,0,0); glTranslatef(0,0,-1.6)
    gluCylinder(gluNewQuadric(),4.0,4.0,3.2,10,3)
    glTranslatef(0,0,1); gluSphere(gluNewQuadric(),1.8,8,5)
    glPopMatrix()
    glPopMatrix()

def draw_vehicle(v):
    if v.is_wreck:
        wx=v.x+v.push_x; wy=v.y+v.push_y
        glPushMatrix()
        glTranslatef(wx,wy,0)
        glRotatef(v.hdeg,0,0,1)
        box_local(0,0,10,v.L,v.W,v.H*0.6,(0.22,0.22,0.22))
        for off in [-v.L*0.3,0,v.L*0.3]:
            box_local(off,0,v.H*0.4,v.L*0.18,v.W*1.05,4,(1.0,0.42,0.05))
        glPopMatrix()
        for i in range(4):
            drift=sim_time*(0.8+i*0.1)+i*2.0
            ox=math.sin(drift)*(4+i*2)
            oy=math.cos(drift*0.6)*(3+i*1.5)
            oz=v.H+sim_time*0.2%30+i*6
            sh=clamp(0.35+i*0.04,0.3,0.6)
            sphere(wx+ox,wy+oy,oz,5+i*1.2,(sh,sh,sh))
        return

    sx=sy=sz=0.0
    if v.shake_t>0:
        dec=v.shake_t/2.0; t=sim_time*30
        sx=math.sin(t*1.7)*v.shake_amp*dec
        sy=math.cos(t*2.1)*v.shake_amp*dec
        sz=abs(math.sin(t*2.8))*v.shake_amp*0.3*dec

    rev_bob=0.0
    if v.rev_timer>0:
        prog=v.rev_timer/1.6
        rev_bob=math.sin(sim_time*22)*prog*1.8

    iz=ix=0.0
    if v.speed<2 and not v.crashed:
        iz=math.sin(v.idle_ph)*0.30+rev_bob; ix=math.cos(v.idle_ph*0.7)*0.12

    cx=v.x+v.push_x; cy=v.y+v.push_y
    relx=camera_pos[0]-cx; rely=camera_pos[1]-cy
    vis=1.0 if (-math.sin(v.heading)*relx+math.cos(v.heading)*rely)>=0 else -1.0

    glPushMatrix()
    glTranslatef(cx+sx+ix,cy+sy,8+sz+iz)
    glRotatef(v.hdeg,0,0,1)

    body=v.color
    roof=(min(body[0]+0.14,1),min(body[1]+0.14,1),min(body[2]+0.14,1))
    win=(0.38,0.68,0.96) if day_mode else (0.18,0.32,0.58)
    hl=(0.55,0.50,0.28) if day_mode else (1.0,0.95,0.70)
    bl=(1.0,0.0,0.0) if v.braking else (0.32,0.0,0.0)

    if v.kind=="bus":
        WY=18.0
        box_local(0,0,15,v.L,v.W,v.H,body)
        box_local(3,0,31,v.L*.72,v.W*.82,9,roof)
        for xw in [-28,-9,10,29]:
            box_local(xw,vis*17,28,10,2,8,win)
        box_local( 43,-9,19,3,5,4,hl); box_local( 43, 9,19,3,5,4,hl)
        box_local(-43,-9,19,3,5,4,bl); box_local(-43, 9,19,3,5,4,bl)
        draw_wheel( 30,vis*WY,v,True)
        draw_wheel(-32,vis*WY,v,False)
    else:
        WY=16.0
        box_local(0,0,11,v.L,v.W,16,body)
        box_local(-5,0,24,30,22,14,roof)
        box_local(12,0,26,8,23,5,win)
        box_local(-22,0,23,5,20,4,win)
        box_local( 4,vis*14,28,14,2,7,win)
        box_local(-12,vis*14,25,10,2,6,win)
        box_local( 28,-8,15,3,5,3,hl); box_local( 28, 8,15,3,5,3,hl)
        box_local(-28,-8,15,3,5,3,bl); box_local(-28, 8,15,3,5,3,bl)
        draw_wheel( 18,vis*WY,v,True)
        draw_wheel(-18,vis*WY,v,False)

    glPopMatrix()

    if v.rev_timer>0:
        prog=v.rev_timer/1.6
        ex=-math.cos(v.heading)*(v.L*0.52)
        ey=-math.sin(v.heading)*(v.L*0.52)
        for i in range(3):
            age=(i/3.0)
            drift=sim_time*6+i*2.1
            ox=ex+math.sin(drift)*3*(1-age)
            oy=ey+math.cos(drift*0.8)*3*(1-age)
            oz=12+age*18
            r=4+age*6
            sh=clamp(0.55+age*0.15,0.4,0.72)
            alpha_fade=prog*(1.0-age*0.6)
            if alpha_fade>0.05:
                sphere(cx+ox,cy+oy,oz,r*(0.4+alpha_fade*0.6),(sh,sh,sh))

    if not day_mode:
        glPushMatrix(); _col((0.52,0.46,0.16))
        glTranslatef(cx,cy,22); glRotatef(v.hdeg,0,0,1); glRotatef(90,0,1,0)
        gluCylinder(gluNewQuadric(),2,24,85,10,3); glPopMatrix()

    if v.crash_timer>0:
        p=clamp(1-v.crash_timer/4.0,0,1)
        for i in range(6):
            drift=sim_time*(1.4+i*0.18)+i*1.9
            ox=math.sin(drift)*(5+i*2)+(i-2)*5
            oy=math.cos(drift*0.73)*(4+i*1.6)
            oz=18+p*44+i*5.5; r=6.5+p*10+i*1.6
            sh=clamp(0.40+p*0.18-i*.025,0.22,0.62)
            sphere(cx+ox,cy+oy,oz,r,(sh,sh,sh))

def draw_road_arrows():
    ac=scene_col((0.95,0.88,0.08),(0.70,0.64,0.06))
    for d in range(4):
        dx,dy=DIR_VEC[d]
        for si in range(3):
            s=-SPAWN_DIST+160+si*160
            if abs(s)<INTER_HALF+40: continue
            x=dx*s; y=dy*s
            slab(x-abs(dy)*4,y-abs(dx)*4,
                 x+abs(dy)*4+dx*20,y+abs(dx)*4+dy*20,
                 1.9,ac,1.1)
            ax=x+dx*24; ay=y+dy*24
            slab(ax-abs(dy)*9,ay-abs(dx)*9,
                 ax+abs(dy)*9+dx*4,ay+abs(dx)*9+dy*4,
                 1.9,ac,1.1)
            ax2=x+dx*34; ay2=y+dy*34
            slab(ax2-abs(dy)*3,ay2-abs(dx)*3,
                 ax2+abs(dy)*3+dx*2,ay2+abs(dx)*3+dy*2,
                 1.9,ac,1.3)

def draw_dashed_x(y,xs,xe,w,col):
    x=xs
    while x<xe:
        x2=min(x+46,xe); mid=(x+x2)*.5
        if abs(mid)>INTER_HALF+12: slab(x,y-w*.5,x2,y+w*.5,1.0,col)
        x+=80

def draw_dashed_y(x,ys,ye,w,col):
    y=ys
    while y<ye:
        y2=min(y+46,ye); mid=(y+y2)*.5
        if abs(mid)>INTER_HALF+12: slab(x-w*.5,y,x+w*.5,y2,1.0,col)
        y+=80

def draw_crosswalks():
    w=scene_col((0.92,0.92,0.88),(0.52,0.56,0.60))
    stripe=8; gap=8
    x=-ROAD_HALF+8
    while x<ROAD_HALF-8:
        slab(x,-STOP_LINE-26,x+stripe,-STOP_LINE-6,1.4,w)
        slab(x, STOP_LINE+6, x+stripe, STOP_LINE+26,1.4,w)
        x+=stripe+gap
    y=-ROAD_HALF+8
    while y<ROAD_HALF-8:
        slab(-STOP_LINE-26,y,-STOP_LINE-6,y+stripe,1.4,w)
        slab( STOP_LINE+6, y, STOP_LINE+26,y+stripe,1.4,w)
        y+=stripe+gap

def draw_roads():
    ground=scene_col((0.08,0.30,0.12),(0.014,0.050,0.065))
    road  =scene_col((0.14,0.14,0.14),(0.036,0.038,0.048))
    inter =scene_col((0.11,0.11,0.11),(0.048,0.048,0.060))
    curb  =scene_col((0.54,0.54,0.54),(0.26,0.26,0.30))
    yellow=scene_col((1.0,0.86,0.05),(0.80,0.66,0.08))
    white =scene_col((0.90,0.90,0.90),(0.62,0.66,0.70))

    slab(-GRID_LEN,-GRID_LEN,GRID_LEN,GRID_LEN,-1.0,ground,2.0)
    slab(-GRID_LEN,-ROAD_HALF,GRID_LEN,ROAD_HALF,0.0,road,1.5)
    slab(-ROAD_HALF,-GRID_LEN,ROAD_HALF,GRID_LEN,0.05,road,1.5)
    slab(-ROAD_HALF,-ROAD_HALF,ROAD_HALF,ROAD_HALF,0.10,inter,1.5)

    for x0,y0,x1,y1 in [
        (-GRID_LEN, ROAD_HALF,   GRID_LEN, ROAD_HALF+7),
        (-GRID_LEN,-ROAD_HALF-7, GRID_LEN,-ROAD_HALF),
        ( ROAD_HALF,-GRID_LEN,   ROAD_HALF+7, GRID_LEN),
        (-ROAD_HALF-7,-GRID_LEN,-ROAD_HALF,   GRID_LEN),
    ]:
        slab(x0,y0,x1,y1,0.9,curb,4.0)

    draw_dashed_x(0,-GRID_LEN,GRID_LEN,5.0,yellow)
    draw_dashed_y(0,-GRID_LEN,GRID_LEN,5.0,yellow)
    draw_dashed_x(-LANE_W,-GRID_LEN,GRID_LEN,3.3,white)
    draw_dashed_x( LANE_W,-GRID_LEN,GRID_LEN,3.3,white)
    draw_dashed_y(-LANE_W,-GRID_LEN,GRID_LEN,3.3,white)
    draw_dashed_y( LANE_W,-GRID_LEN,GRID_LEN,3.3,white)

    slab(-STOP_LINE-3,-ROAD_HALF,-STOP_LINE+3, 0,      1.3,white)
    slab( STOP_LINE-3, 0,         STOP_LINE+3, ROAD_HALF,1.3,white)
    slab( 0,          -STOP_LINE-3, ROAD_HALF,-STOP_LINE+3,1.3,white)
    slab(-ROAD_HALF,   STOP_LINE-3, 0,          STOP_LINE+3,1.3,white)

    draw_crosswalks()
    draw_road_arrows()

BLDS=[
    (-440,380,0, 90,100,165, 0.40,0.42,0.52),
    (-305,365,0, 72,125,115, 0.50,0.38,0.32),
    (-435,-355,0,125, 78,148,0.36,0.38,0.50),
    (-285,-365,0, 82, 88, 92,0.52,0.42,0.36),
    ( 315, 360,0, 98,108,132,0.46,0.46,0.54),
    ( 445, 315,0, 78, 98,184,0.36,0.40,0.56),
    ( 345,-338,0,112, 82,108,0.50,0.38,0.34),
    ( 475,-385,0, 78, 88,138,0.42,0.44,0.48),
    (-440, 260,0, 60, 70, 80,0.44,0.40,0.36),
    ( 315,-240,0, 65, 72, 95,0.38,0.44,0.50),
    ( 440, 240,0, 55, 65, 70,0.50,0.46,0.40),
    (-305,-260,0, 68, 60, 85,0.42,0.40,0.46),
]

def draw_buildings():
    for b in BLDS:
        cx,cy,zb,sx,sy,sz=b[0],b[1],b[2],b[3],b[4],b[5]
        dc=(b[6],b[7],b[8])
        nc=(b[6]*.28,b[7]*.28,b[8]*.28)
        col=scene_col(dc,nc)
        box_world(cx,cy,zb,sx,sy,sz,col)
        wc=scene_col((0.78,0.88,1.00),(1.0,0.78,0.22))
        rz=30
        while rz<sz-14:
            for ci in range(max(1,int(sx//22))):
                wx2=cx-sx*.5+8+ci*22
                box_world(wx2,cy-sy*.51,rz,10,2,8,wc)
            for ci in range(max(1,int(sy//22))):
                wy2=cy-sy*.5+8+ci*22
                box_world(cx-sx*.51,wy2,rz,2,10,8,wc)
            rz+=28
        box_world(cx,cy,sz+2,sx*.15,sy*.15,12,(b[6]*.55,b[7]*.55,b[8]*.55))

def draw_trees():
    tp=[(-160,-160),(160,-160),(-160,160),(160,160),
        (-520,182),(520,-182),(182,520),(-182,-520),
        (-340,96),(340,-96),(96,340),(-96,-340),
        (-520,-182),(520,182),(-182,520),(182,-520)]
    tc=scene_col((0.38,0.20,0.08),(0.15,0.09,0.04))
    lc=scene_col((0.05,0.46,0.13),(0.014,0.15,0.07))
    for tx,ty in tp:
        cyl_z(tx,ty,0,6,36,tc)
        sphere(tx,ty,50,24,lc); sphere(tx+8,ty+6,40,16,lc)

def draw_streetlamps():
    lamps=[(-162,-162),(162,-162),(-162,162),(162,162),
           (-472,106),(472,-106),(106,472),(-106,-472),
           (-312,92),(312,-92),(92,312),(-92,-312)]
    pc=scene_col((0.34,0.34,0.36),(0.18,0.18,0.22))
    hc=scene_col((0.22,0.22,0.24),(0.10,0.10,0.14))
    ob=(0.46,0.46,0.38); on=(1.0,0.90,0.38)
    for lx,ly in lamps:
        cyl_z(lx,ly,0,3.0,60,pc)
        box_world(lx,   ly,60,11,11,4,pc)
        box_world(lx+9, ly,63,18,5, 4,pc)
        box_world(lx+18,ly,60,14,12,6,hc)
        if day_mode:
            sphere(lx+18,ly,57,5.0,ob)
        else:
            sphere(lx+18,ly,57,5.8,on)
            cone_z(lx+18,ly,4,30,4,52,(0.42,0.34,0.09))
            slab(lx+18-34,ly-17,lx+18+34,ly+17,0.6,(0.32,0.26,0.07),0.9)

def draw_sky_body():
    if day_mode:
        sphere(-500,520,430,42,(1.0,0.88,0.26))
    else:
        sphere(-500,520,430,30,(0.72,0.82,1.0))
        for i in range(24):
            sx2=(i*137.5)%GRID_LEN-GRID_LEN*.5
            sy2=(i*93.7 )%GRID_LEN-GRID_LEN*.5
            sphere(sx2,sy2,520,4,(0.90,0.90,0.96))

def draw_pedestrians():
    if not ped_active: return
    phase=(sim_time%3.5)/3.5
    for i in range(5):
        t=(phase+i*0.2)%1.0
        px=-ROAD_HALF+t*(ROAD_HALF*2)
        py=STOP_LINE+16+(i%2)*8
        pz=2.5
        cyl_z(px,py,pz,3.0,18,(0.15,0.20,0.85))
        sphere(px,py,pz+23,5.2,(0.94,0.72,0.52))
        sw=math.sin(sim_time*4+i)
        box_world(px+sw*1.5,py,pz+10,3,2,12,(0.15,0.20,0.78))
    for i in range(3):
        t=(1.0-phase+i*0.33)%1.0
        px=-ROAD_HALF+t*(ROAD_HALF*2)
        py=-STOP_LINE-16-(i%2)*8
        pz=2.5
        cyl_z(px,py,pz,3.0,18,(0.80,0.40,0.10))
        sphere(px,py,pz+23,5.2,(0.94,0.72,0.52))

def draw_blockage():
    if not block_active: return
    bx,by=lane_pt(BLOCK_DIR,BLOCK_LANE,BLOCK_LONG)
    box_world(bx,by,0,64,10,16,(1.0,0.24,0.04))
    box_world(bx,by+11,0,50,8,11,(0.94,0.94,0.94))
    for off in [-28,0,28]:
        glPushMatrix(); _col((1.0,0.40,0.05))
        glTranslatef(bx+off,by-14,0)
        gluCylinder(gluNewQuadric(),7,2,26,10,5); glPopMatrix()

def setup_camera():
    global camera_pos,smooth_eye,smooth_look
    glMatrixMode(GL_PROJECTION); glLoadIdentity()
    gluPerspective(fovY,WINDOW_W/WINDOW_H,1.0,4000)
    glMatrixMode(GL_MODELVIEW); glLoadIdentity()

    if cam_mode=="DRIVER" and vehicles:
        idx=min(follow_idx,len(vehicles)-1)
        v=vehicles[idx]
        back=150 if v.kind=="car" else 185
        up  = 80 if v.kind=="car" else 95
        ex=v.x-math.cos(v.heading)*back
        ey=v.y-math.sin(v.heading)*back
        ez=up+math.sin(v.idle_ph)*0.4
        lx=v.x+math.cos(v.heading)*290
        ly=v.y+math.sin(v.heading)*290; lz=30.0
    else:
        rad=math.radians(cam_angle%360)
        ex=math.cos(rad)*cam_radius; ey=math.sin(rad)*cam_radius
        ez=cam_height; lx=ly=0.0; lz=22.0

    if cam_mode=="TOP":
        smooth_eye=(ex,ey,ez); smooth_look=(lx,ly,lz)
    elif smooth_eye is None:
        smooth_eye=(ex,ey,ez); smooth_look=(lx,ly,lz)
    else:
        k=0.18
        smooth_eye =tuple(smooth_eye[i] +((ex,ey,ez)[i]-smooth_eye[i])*k  for i in range(3))
        smooth_look=tuple(smooth_look[i]+((lx,ly,lz)[i]-smooth_look[i])*k for i in range(3))

    camera_pos=smooth_eye
    gluLookAt(*smooth_eye,*smooth_look,0,0,1)

def congestion(q):
    return "LOW" if q<=1 else "MED" if q<=4 else "HIGH"

def build_buttons():
    global BUTTONS
    BUTTONS=[]
    pw=240
    px=WINDOW_W-pw-10
    y=WINDOW_H-90
    bw=pw-20
    hw=(bw-8)//2
    bh=30
    gap=8
    BUTTONS.append((px+10,y,px+10+bw,y+bh,"AUTO/MANUAL","auto"))
    y-=bh+gap+4
    BUTTONS.append((px+10,y,px+10+hw,y+bh,"N GREEN",1))
    BUTTONS.append((px+10+hw+8,y,px+10+bw,y+bh,"S GREEN",3))
    y-=bh+gap
    BUTTONS.append((px+10,y,px+10+hw,y+bh,"E GREEN",0))
    BUTTONS.append((px+10+hw+8,y,px+10+bw,y+bh,"W GREEN",2))
    y-=bh+gap+4
    BUTTONS.append((px+10,y,px+10+hw,y+bh,"+ CAR","add"))
    BUTTONS.append((px+10+hw+8,y,px+10+bw,y+bh,"- CAR","remove"))
    y-=bh+gap+4
    BUTTONS.append((px+10,y,px+10+bw,y+bh,"CAMERA","camera"))
    y-=bh+gap
    BUTTONS.append((px+10,y,px+10+bw,y+bh,"DAY / NIGHT","daynight"))
    y-=bh+gap
    BUTTONS.append((px+10,y,px+10+bw,y+bh,"INCIDENT","incident"))
    y-=bh+gap+4
    BUTTONS.append((px+10,y,px+10+hw,y+bh,"PAUSE","pause"))
    BUTTONS.append((px+10+hw+8,y,px+10+bw,y+bh,"RESET","reset"))

def btn_col(action):
    if action=="auto": return (0.05,0.35,0.15) if traffic_auto else (0.35,0.15,0.05)
    if isinstance(action,int):
        s=dir_state[action]
        if s=="GREEN": return (0.05,0.35,0.1)
        if s=="YELLOW": return (0.4,0.3,0.05)
        return (0.35,0.05,0.05)
    if action=="pause": return (0.05,0.35,0.1) if paused else (0.35,0.25,0.05)
    if action=="camera": return (0.05,0.2,0.4) if cam_mode=="TOP" else (0.35,0.15,0.05)
    if action=="daynight": return (0.05,0.3,0.45) if day_mode else (0.1,0.1,0.25)
    if action=="incident": return (0.4,0.05,0.05) if manual_incident else (0.1,0.2,0.3)
    if action=="reset": return (0.35,0.05,0.05)
    if action in ("add","remove"): return (0.1,0.2,0.35)
    return (0.1,0.2,0.3)

def btn_border_col(action):
    if isinstance(action,int):
        s=dir_state[action]
        if s=="GREEN": return (0.2,0.8,0.3)
        if s=="YELLOW": return (0.8,0.7,0.2)
        return (0.8,0.2,0.2)
    if action=="auto": return (0.2,0.8,0.4) if traffic_auto else (0.8,0.4,0.2)
    return (0.3,0.5,0.7)

def draw_hud():
    moving=stopped=crashed=0
    for v in vehicles:
        if v.is_wreck: continue
        if v.speed>3: moving+=1
        if v.stopped: stopped+=1
        if v.crashed: crashed+=1
    flow=completed_count/(sim_time/60) if sim_time>1 else 0.0
    avgw=total_wait/completed_count if completed_count else 0.0
    mode="AUTO" if traffic_auto else "MANUAL"
    sig_t=f"{sig_remaining():.1f}"
    lt="DAY" if day_mode else "NIGHT"

    panel_w=240
    panel_x=WINDOW_W-panel_w-10
    panel_top=WINDOW_H-30
    panel_bot=100

    enter_2d()

    hud_box(0,WINDOW_H-30,WINDOW_W,WINDOW_H,(0.02,0.03,0.05),0.85)

    hud_rounded(10,WINDOW_H-136,570,WINDOW_H-36,(0.05,0.06,0.08),0.85)
    hud_border(10,WINDOW_H-136,570,WINDOW_H-36,(0.2,0.3,0.5),0.6)

    hud_rounded(panel_x,panel_bot,panel_x+panel_w,panel_top,(0.05,0.06,0.08),0.9)
    hud_border(panel_x,panel_bot,panel_x+panel_w,panel_top,(0.2,0.3,0.5),0.6,1.5)

    title_y=panel_top-28
    hud_rounded(panel_x+10,title_y-6,panel_x+panel_w-10,panel_top-6,(0.1,0.15,0.25),0.9)

    for b in BUTTONS:
        x0,y0,x1,y1,lbl,action=b
        bc=btn_col(action)
        hud_rounded(x0,y0,x1,y1,bc,0.9)
        brd=btn_border_col(action)
        hud_border(x0,y0,x1,y1,brd,0.8,1.2)

    sig_colors={"GREEN":(0.1,0.7,0.2),"YELLOW":(0.8,0.6,0.1),"RED":(0.8,0.1,0.1)}
    sx0=panel_x+10; sy_bar=panel_bot+10
    bar_h=18; bar_w=(panel_w-30)//4
    for idx,(lbl,d) in enumerate([("N",1),("E",0),("S",3),("W",2)]):
        bx=sx0+idx*(bar_w+3)
        sc=sig_colors[dir_state[d]]
        hud_rounded(bx,sy_bar,bx+bar_w,sy_bar+bar_h,sc,0.9)
        hud_border(bx,sy_bar,bx+bar_w,sy_bar+bar_h,(1,1,1),0.2)

    if paused:
        hud_rounded(WINDOW_W//2-88,WINDOW_H//2-28,WINDOW_W//2+88,WINDOW_H//2+28,
                    (0.55,0.18,0.02),0.92)
        hud_border(WINDOW_W//2-88,WINDOW_H//2-28,WINDOW_W//2+88,WINDOW_H//2+28,
                   (1.0,0.7,0.2),0.90,2.0)

    exit_2d()

    draw_text(18,WINDOW_H-19,
        "Arrows/A-D: orbit   W/S: zoom   C: camera   TAB: follow   N: night   1-4: signal   M: mode   E: incident   SPACE: pause   R: reset",
        GLUT_BITMAP_HELVETICA_12,(0.7,0.85,1.0))

    draw_text(18,WINDOW_H-60,
        f"Mode: {mode}   Signal: {sig_t}s   {lt}   Ped: {'ON' if ped_active else 'OFF'}   Incident: {'ON' if block_active else 'OFF'}",
        GLUT_BITMAP_HELVETICA_12,(0.85,0.95,0.75))
    draw_text(18,WINDOW_H-78,
        f"Vehicles: {len(vehicles)}   Moving: {moving}   Stopped: {stopped}   Wrecks: {len(wrecks)}   Crashes: {crash_count}   Done: {completed_count}",
        GLUT_BITMAP_HELVETICA_12,(0.8,0.85,1.0))
    draw_text(18,WINDOW_H-96,
        f"Flow: {flow:.1f}/min   Avg Wait: {avgw:.1f}s",
        GLUT_BITMAP_HELVETICA_12,(0.75,0.8,0.9))
    draw_text(18,WINDOW_H-116,
        f"Queue:   E:{queue_counts[0]} ({congestion(queue_counts[0])})   "
        f"N:{queue_counts[1]} ({congestion(queue_counts[1])})   "
        f"W:{queue_counts[2]} ({congestion(queue_counts[2])})   "
        f"S:{queue_counts[3]} ({congestion(queue_counts[3])})",
        GLUT_BITMAP_HELVETICA_12,(1.0,0.8,0.3))

    draw_text(panel_x+46,panel_top-23,"CONTROL PANEL",
              GLUT_BITMAP_HELVETICA_18,(0.9,0.95,1.0))

    for b in BUTTONS:
        x0,y0,x1,y1,lbl,action=b
        shown=lbl
        if action=="auto":
            shown="AUTO MODE" if traffic_auto else "MANUAL MODE"
            tc=(0.6,1.0,0.6) if traffic_auto else (1.0,0.7,0.5)
        elif isinstance(action,int):
            st=dir_state[action]
            dir_label=BTN_DIRS[[d for _,d in BTN_DIRS].index(action)][0]
            shown=f"{dir_label}: {st}"
            tc={"GREEN":(0.4,1.0,0.4),"YELLOW":(1.0,1.0,0.4),"RED":(1.0,0.4,0.4)}[st]
        elif action=="pause":
            shown="RESUME" if paused else "PAUSE"
            tc=(0.6,1.0,0.6) if paused else (1.0,0.8,0.4)
        elif action=="camera":
            shown="FOLLOW CAM" if cam_mode=="DRIVER" else "TOP VIEW"
            tc=(1.0,0.8,0.5)
        elif action=="daynight":
            shown="DAY MODE" if day_mode else "NIGHT MODE"
            tc=(1.0,0.9,0.6) if day_mode else (0.6,0.8,1.0)
        elif action=="incident":
            shown="INCIDENT ON" if manual_incident else "INCIDENT OFF"
            tc=(1.0,0.5,0.3) if manual_incident else (0.7,0.8,1.0)
        elif action=="reset":
            shown="RESET"; tc=(1.0,0.5,0.5)
        elif action=="add":
            shown="+ ADD CAR"; tc=(0.6,0.9,1.0)
        elif action=="remove":
            shown="- REM CAR"; tc=(1.0,0.7,0.5)
        else:
            tc=(1.0,1.0,1.0)
        
        bw_current=x1-x0
        x_off=65 if bw_current>150 else 22
        draw_text(x0+x_off,y0+10,shown,GLUT_BITMAP_HELVETICA_12,tc)

    sig_lbl_col={"GREEN":(0.4,1.0,0.5),"YELLOW":(1.0,1.0,0.4),"RED":(1.0,0.4,0.4)}
    for idx,(lbl,d) in enumerate([("N",1),("E",0),("S",3),("W",2)]):
        bx=sx0+idx*(bar_w+3)
        sc=sig_lbl_col[dir_state[d]]
        draw_text(bx+(bar_w//2)-5,panel_bot+34,f"{lbl}",GLUT_BITMAP_HELVETICA_12,sc)

    if paused:
        draw_text(WINDOW_W//2-44,WINDOW_H//2-4,"PAUSED",
                  GLUT_BITMAP_TIMES_ROMAN_24,(1.0,0.90,0.40))

def update_events():
    global ped_active,block_active
    cyc=sim_time%32; ped_active=14<=cyc<=18
    bc=sim_time%50;  block_active=manual_incident or (32<=bc<=42)

def show_screen():
    sky=scene_col((0.44,0.62,0.88),(0.02,0.04,0.12))
    glClearColor(*sky,1.0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glViewport(0,0,WINDOW_W,WINDOW_H)
    glLoadIdentity()
    setup_camera()

    draw_sky_body()
    draw_roads()
    draw_buildings()
    draw_trees()
    draw_streetlamps()
    draw_blockage()
    draw_pedestrians()

    for w in wrecks:
        draw_vehicle(w)

    ordered=sorted(vehicles,
        key=lambda v:(v.x-camera_pos[0])**2+(v.y-camera_pos[1])**2,reverse=True)
    for v in ordered:
        draw_vehicle(v)

    for d in range(4):
        draw_traffic_light(d)

    draw_hud()
    glutSwapBuffers()

def update_sim(dt):
    global spawn_timer,completed_count,total_wait,sim_time,day_night_t

    if paused: return
    dt=clamp(dt,0,0.055)
    sim_time+=dt

    target_t=0.0 if day_mode else 1.0
    day_night_t=lerp(day_night_t,target_t,dt*1.4)

    update_events()
    update_signal(dt)

    if spawn_on:
        spawn_timer+=dt
        if spawn_timer>=SPAWN_INTERVAL: spawn_timer=0.0; try_spawn()

    for v in vehicles:
        if v.is_wreck: continue
        _update_pose(v)
        v.wroll=(v.wroll+v.speed*dt*(360/44.0))%360
        rev_speed= 14 if v.rev_timer>0 else 3
        v.idle_ph+=dt*(rev_speed if v.stopped else 3)
        if v.rev_timer>0: v.rev_timer=max(0,v.rev_timer-dt)

    for v in vehicles:
        if v.is_wreck: continue

        if v.crash_timer>0:
            v.crash_timer=max(0,v.crash_timer-dt)
            v.speed=max(0,v.speed-v.brake*1.5*dt)
            if v.crash_timer<=0:
                promote_to_wreck(v)
            continue

        if v.shake_t>0:
            v.shake_t=max(0,v.shake_t-dt)

        tgt,stop_at=desired_speed(v)
        if v.speed>tgt:
            v.speed-=v.brake*dt
            if v.speed<tgt: v.speed=tgt
        else:
            v.speed+=v.accel*dt
            if v.speed>tgt: v.speed=tgt
        v.speed=clamp(v.speed,0,v.max_v)
        v.braking=v.speed<tgt-0.4 or v.speed<3

        old_f=v.dist+v.L*0.48
        nd=v.dist+v.speed*dt
        nf=nd+v.L*0.48
        if stop_at is not None and old_f<=stop_at and nf>stop_at:
            nd=stop_at-v.L*0.48; v.speed=0
        v.dist=max(0,nd)
        v.stopped=v.speed<2 and v.dist<v.path["len"]-40
        if v.stopped: v.wait_t+=dt
        v.push_x*=max(0,1-3*dt); v.push_y*=max(0,1-3*dt)
        _update_pose(v)
        if v.dist>=v.path["len"]-4: v.completed=True

    collision_step()

    survivors=[]
    for v in vehicles:
        if v.is_wreck: continue
        if v.completed:
            if not v.crashed:
                completed_count+=1; total_wait+=v.wait_t
        else:
            survivors.append(v)
    vehicles[:]=survivors
    count_queues()

def toggle_cam():
    global cam_mode,smooth_eye,smooth_look,follow_idx
    cam_mode="DRIVER" if cam_mode=="TOP" else "TOP"
    if cam_mode=="DRIVER" and vehicles:
        for i,v in enumerate(vehicles):
            if not v.is_wreck and not v.crashed: follow_idx=i; break
    smooth_eye=smooth_look=None

def handle_button(mx,my):
    global paused,manual_incident,day_mode,follow_idx,cam_mode,smooth_eye,smooth_look
    for b in BUTTONS:
        x0,y0,x1,y1,lbl,action=b
        if x0-4<=mx<=x1+4 and y0-4<=my<=y1+4:
            if action=="auto":    toggle_auto()
            elif isinstance(action,int): request_manual_green(action)
            elif action=="add":   try_spawn()
            elif action=="remove":
                if vehicles: vehicles.pop()
            elif action=="camera":toggle_cam()
            elif action=="daynight": day_mode=not day_mode
            elif action=="incident": manual_incident=not manual_incident
            elif action=="pause": paused=not paused
            elif action=="reset": reset_sim()
            return True
    return False

def keyboardListener(key,x,y):
    global paused,cam_radius,cam_angle,cam_height,day_mode
    global manual_incident,spawn_on,cam_mode,smooth_eye,smooth_look,follow_idx
    k=key.lower()
    if k==b'p' or key==b' ': paused=not paused
    elif k==b'r': reset_sim()
    elif k==b'w': cam_radius=max(200,cam_radius-45)
    elif k==b's': cam_radius=min(1500,cam_radius+45)
    elif k==b'a': cam_angle=(cam_angle+4)%360
    elif k==b'd': cam_angle=(cam_angle-4)%360
    elif k==b'm': toggle_auto()
    elif k==b'n': day_mode=not day_mode
    elif k==b'c': toggle_cam()
    elif key==b'\t':
        live=[i for i,v in enumerate(vehicles) if not v.is_wreck and not v.crashed]
        if live:
            follow_idx=live[(live.index(follow_idx)+1)%len(live)] if follow_idx in live else live[0]
            cam_mode='DRIVER'; smooth_eye=smooth_look=None
    elif key==b'\x1b': cam_mode="TOP"
    elif k==b'e': manual_incident=not manual_incident
    elif k==b'v': spawn_on=not spawn_on
    elif k==b'+': [try_spawn() for _ in range(2)]
    elif k==b'-':
        live=[v for v in vehicles if not v.is_wreck]
        if live: vehicles.remove(live[-1])
    elif k==b'1': request_manual_green(1)
    elif k==b'2': request_manual_green(0)
    elif k==b'3': request_manual_green(3)
    elif k==b'4': request_manual_green(2)

def specialKeyListener(key,x,y):
    global cam_angle,cam_height
    if key==GLUT_KEY_LEFT:  cam_angle=(cam_angle+4)%360
    if key==GLUT_KEY_RIGHT: cam_angle=(cam_angle-4)%360
    if key==GLUT_KEY_UP:    cam_height=min(1200,cam_height+28)
    if key==GLUT_KEY_DOWN:  cam_height=max(80, cam_height-28)

def mouseListener(button,state,mx,my):
    gl_y=WINDOW_H-my
    if button==GLUT_LEFT_BUTTON and state==GLUT_DOWN:
        handle_button(mx,gl_y)
    if button==GLUT_RIGHT_BUTTON and state==GLUT_DOWN:
        toggle_cam()

def idle():
    global last_wall
    now=time.time()
    if last_wall is None: last_wall=now
    update_sim(now-last_wall); last_wall=now
    glutPostRedisplay()

def reset_sim():
    global vehicles,wrecks,completed_count,total_wait,sim_time
    global auto_timer,auto_phase,auto_green_dir,dir_state,dir_timer,dir_prev_state
    global spawn_timer,_seed,manual_incident,crash_count
    global traffic_auto,last_wall,cam_mode,smooth_eye,smooth_look
    global day_mode,day_night_t,follow_idx

    vehicles=[]; wrecks=[]
    completed_count=0; total_wait=0.0; sim_time=0.0
    auto_timer=0.0; auto_phase="GREEN"; auto_green_dir=1
    dir_state=["RED","GREEN","RED","RED"]; dir_timer=[0.0]*4
    dir_prev_state=["RED","GREEN","RED","RED"]
    spawn_timer=0.0; _seed=423; manual_incident=False
    traffic_auto=True; crash_count=0; last_wall=None
    cam_mode="TOP"; day_mode=True; day_night_t=0.0
    smooth_eye=smooth_look=None; follow_idx=0

    for route in [(0,"straight",0,70),(0,"right",1,250),
                  (1,"straight",0,105),(1,"left",0,285),
                  (2,"straight",0,80),(2,"right",1,255),
                  (3,"straight",0,115),(3,"left",0,300)]:
        add_vehicle(*route)
    count_queues()

def main():
    build_buttons()
    reset_sim()
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH)
    glutInitWindowSize(WINDOW_W,WINDOW_H)
    glutInitWindowPosition(0,0)
    glutCreateWindow(b"3D Intelligent Traffic Simulator  v4  |  CSE423 Group 7")
    glutDisplayFunc(show_screen)
    glutKeyboardFunc(keyboardListener)
    glutSpecialFunc(specialKeyListener)
    glutMouseFunc(mouseListener)
    glutIdleFunc(idle)
    glutMainLoop()

if __name__=="__main__":
    main()
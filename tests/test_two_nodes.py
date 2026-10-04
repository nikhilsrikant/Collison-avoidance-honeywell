import subprocess, threading, time, os, sys, select
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tabletop
class PipeSerial:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.q = []
    def read(self, n):
        r, _, _ = select.select([self.p.stdout], [], [], 0.05)
        return os.read(self.p.stdout.fileno(), n) if r else b""
    def write(self, b):
        self.p.stdin.write(b)
    def close(self): self.p.kill()
def mk(exe, name):
    s = PipeSerial('/tmp/mock/' + exe)
    link = tabletop.NodeLink(name, s, exe, {})
    return link, s
def st(link):
    return dict(link.status or {})
def wait_for(cond, timeout, link):
    t = time.time()
    while time.time() - t < timeout:
        if cond(st(link)): return round(time.time() - t, 2)
        time.sleep(0.05)
    return None
class Feeder(threading.Thread):
    def __init__(self, link): super().__init__(daemon=True); self.link = link; self.line = None
    def run(self):
        while True:
            if self.line: self.link.write(self.line)
            time.sleep(0.2)
ok = lambda c, m: print(('PASS ' if c else 'FAIL ') + m)

print('=== A: our plane vs UNEQUIPPED jet')
n1, s1 = mk('node1', 'ours'); jt, sj = mk('node2jet', 'jet')
relay = tabletop.RadioRelay(0.0); relay.add(n1); relay.add(jt)
time.sleep(3.0)
ok(n1.hello.get('eq') == 1 and jt.hello.get('eq') == 0, 'HELLO tells the boards apart: %s / %s' % (n1.hello, jt.hello))
ok(2 in n1.peers and n1.peers[2]['eq'] == 0, 'our node hears the jet beacon, equipped=0')
f = Feeder(n1); f.start()
f.line = 'T 1 35 H S'; ok(wait_for(lambda s: s.get('stage') == 1, 2, n1) is not None, 'traffic advisory -> stage 1')
f.line = 'T 2 24 D R'; ok(wait_for(lambda s: s.get('stage') == 2, 2, n1) is not None, 'resolution -> stage 2')
s = st(n1); ok(s['vert'] == 'D' and s['turn'] == 'R' and s['neg'] == '-', 'unilateral, uses laptop sense D/R: %s' % s)
t3 = wait_for(lambda s: s.get('stage') == 3, 5, n1); ok(t3 is not None and t3 > 2.0, 'pilot not following -> buzzer stage 3 after ~3 s (%s s)' % t3)
s1.write(b'#Q\n'); time.sleep(0.3)
t4 = wait_for(lambda s: s.get('stage') == 4, 4, n1); ok(t4 is not None, 'still not following -> stage 4 shaker + ACK (%s s later)' % t4)
n1.write('ACK'); ok(wait_for(lambda s: s.get('ack') == 1, 1, n1) is not None, 'ACK from display acknowledged')
s1.write(b'#P -9\n'); ok(wait_for(lambda s: s.get('stage') == 2 and s.get('comply') == 1, 3, n1) is not None, 'pilot pitches down -> following, back to stage 2')
s1.write(b'#P 0\n'); t = time.time()
t3b = wait_for(lambda s: s.get('stage', 0) >= 3, 5, n1); s = st(n1)
ok(t3b is not None and 2.7 < t3b < 3.5 and s.get('stage') == 3, 'stops following -> fresh 3 s before the buzzer, not straight to shaker (%s s, stage %s)' % (t3b, s.get('stage')))
n1.write('AP 1')
ta = wait_for(lambda s: s.get('auto') == 1, 8, n1); ok(ta is not None, 'autopilot engaged + not following -> AUTO avoid (%s s)' % ta)
s = st(n1); ok(s.get('ap') == 1, 'AP flag set')
s1.write(b'#P 25\n'); ok(wait_for(lambda s: s.get('auto') == 0, 2, n1) is not None, 'pilot pushes against it -> autopilot off (override)')
f.line = 'T 0 255 H S'; tc = wait_for(lambda s: s.get('stage') == 0, 4, n1); ok(tc is not None, 'clear -> stage 0 (%s s)' % tc)
print('   node events:', list(n1.events)[-6:])
s1.close(); sj.close(); time.sleep(0.3)

print('=== A2: autopilot OFF never flies')
n1, s1 = mk('node1', 'ours'); time.sleep(2.8); f = Feeder(n1); f.start(); f.line = 'T 2 8 C R'
time.sleep(7); s = st(n1); ok(s.get('stage') == 4 and s.get('auto') == 0, 'stage %s, auto %s with AP off' % (s.get('stage'), s.get('auto')))
s1.close(); time.sleep(0.3)

print('=== B: both EQUIPPED, only our plane has a laptop')
n1, s1 = mk('node1', 'ours'); n2, s2 = mk('node2eq', 'jet')
relay = tabletop.RadioRelay(0.0); relay.add(n1); relay.add(n2)
time.sleep(3.0); f = Feeder(n1); f.start(); f.line = 'T 2 24 D L'   # laptop's sense must be IGNORED vs equipped peer
wait_for(lambda s: s.get('neg') == 'A', 3, n1); time.sleep(0.5)
a, b = st(n1), st(n2)
ok(a.get('vert') == 'C' and b.get('vert') == 'D', 'lower ID climbs, higher descends: ours %s, jet %s' % (a.get('vert'), b.get('vert')))
ok(a.get('neg') == 'A' and b.get('neg') == 'R', 'PROPOSE / ACK exchanged: ours %s, jet %s' % (a.get('neg'), b.get('neg')))
ok(b.get('stage', 0) >= 2, 'jet with no laptop still raised its own resolution from the proposal')
s1.close(); s2.close(); time.sleep(0.3)

print('=== C: both equipped, both see the threat, radio 100% LOST')
n1, s1 = mk('node1', 'ours'); n2, s2 = mk('node2eq', 'jet')
relay = tabletop.RadioRelay(0.0); relay.add(n1); relay.add(n2)
time.sleep(3.0); relay.loss = 1.0   # peers already heard each other (equipped), then the link dies
f1 = Feeder(n1); f1.start(); f2 = Feeder(n2); f2.start()
f1.line = 'T 2 20 C R'; f2.line = 'T 2 20 C R'      # both laptops would pick CLIMB: the rule must split them
wait_for(lambda s: s.get('neg') == 'U', 3, n1); time.sleep(0.6)
a, b = st(n1), st(n2)
ok(a.get('vert') == 'C' and b.get('vert') == 'D', 'opposite senses with no radio: ours %s, jet %s (neg %s/%s)' % (a.get('vert'), b.get('vert'), a.get('neg'), b.get('neg')))
print('   relay sent %d, dropped %d' % (relay.sent, relay.dropped))
s1.close(); s2.close()

#!/usr/bin/env python3
"""
Panda3D ball-in-maze demo controlled by MacBook tilt via spu_sensor.

Run:
  sudo python3 tilt_maze.py
"""

import math
import multiprocessing
import multiprocessing.shared_memory
import os
import signal
import sys
import time

from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight, BitMask32, CollisionHandlerQueue, CollisionNode,
    CollisionRay, CollisionTraverser, DirectionalLight,
    LRotationf, LVector3, Material, TextNode, loadPrcFileData,
)
from direct.gui.OnscreenText import OnscreenText
from direct.interval.FunctionInterval import Func, Wait
from direct.interval.LerpInterval import LerpFunc
from direct.interval.MetaInterval import Parallel, Sequence
from direct.task.Task import Task

from spu_sensor import SHM_SIZE, sensor_worker, shm_read_new


TILT_SHM_NAME = "tilt_maze_shm"
ACCEL = 70
MAX_SPEED = 5
MAX_SPEED_SQ = MAX_SPEED ** 2
TILT_SCALE = 18.0   # degrees of maze tilt per 1g

loadPrcFileData("", "win-size 1280 960")
loadPrcFileData("", "framebuffer-multisample 1")
loadPrcFileData("", "multisamples 4")

# Add ball-in-maze models directory to search path
_MAZE_DIR = os.path.dirname(os.path.abspath(__file__))
loadPrcFileData("", f"model-path {_MAZE_DIR}")


class TiltSensor:
    def __init__(self):
        self.enabled = True
        self.shm = None
        self.worker = None
        self.last_total = 0
        self.latest = (0.0, 0.0, 1.0)
        self.last_sample_at = 0.0
        self.restarts = 0
        self.next_restart_at = 0.0

    def start(self):
        if not self.enabled:
            return
        try:
            old = multiprocessing.shared_memory.SharedMemory(name=TILT_SHM_NAME, create=False)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        self.shm = multiprocessing.shared_memory.SharedMemory(
            name=TILT_SHM_NAME, create=True, size=SHM_SIZE
        )
        for i in range(SHM_SIZE):
            self.shm.buf[i] = 0
        self._start_worker()

    def _start_worker(self):
        self.worker = multiprocessing.Process(
            target=sensor_worker, args=(TILT_SHM_NAME, self.restarts), daemon=True
        )
        self.worker.start()
        self.next_restart_at = time.time() + 0.5

    def poll(self):
        if not self.enabled or self.shm is None:
            return self.latest
        if (self.worker is None or not self.worker.is_alive()) and time.time() >= self.next_restart_at:
            self.restarts += 1
            self._start_worker()
        samples, self.last_total = shm_read_new(self.shm.buf, self.last_total)
        if samples:
            self.latest = (
                sum(s[0] for s in samples) / len(samples),
                sum(s[1] for s in samples) / len(samples),
                sum(s[2] for s in samples) / len(samples),
            )
            self.last_sample_at = time.time()
        return self.latest

    def has_recent_data(self, max_age=0.35):
        if not self.enabled:
            return False
        return (time.time() - self.last_sample_at) <= max_age

    def stop(self):
        if self.worker and self.worker.is_alive():
            self.worker.kill()
            self.worker.join(timeout=1.0)
        self.worker = None
        if self.shm:
            self.shm.close()
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass
            self.shm = None


class TiltMazeGame(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        self.disableMouse()
        camera.setPosHpr(0, 0, 25, 0, -90, 0)

        self.title = OnscreenText(
            text="Panda3D: Ball in Maze — tilt to play",
            parent=base.a2dBottomRight, align=TextNode.ARight,
            fg=(1, 1, 1, 1), pos=(-0.1, 0.1), scale=.07,
            shadow=(0, 0, 0, 0.5),
        )

        self.sensor = TiltSensor()
        self.sensor.start()

        hint = "Tilt your MacBook to roll the ball  |  arrow keys also work  |  Esc: quit"

        self.instructions = OnscreenText(
            text=hint,
            parent=base.a2dTopLeft, align=TextNode.ALeft,
            pos=(0.05, -0.08), fg=(1, 1, 1, 1), scale=.055,
            shadow=(0, 0, 0, 0.5),
        )
        self.status = OnscreenText(
            text="",
            parent=base.a2dTopLeft, align=TextNode.ALeft,
            pos=(0.05, -0.16), fg=(0.8, 0.9, 1, 1), scale=.048,
            shadow=(0, 0, 0, 0.5),
        )

        self.accept("escape", self._quit)

        # Keyboard fallback
        self.keys = {}
        for key in ("arrow_up", "arrow_down", "arrow_left", "arrow_right"):
            self.accept(key, self._set_key, [key, True])
            self.accept(f"{key}-up", self._set_key, [key, False])

        # Low-pass tilt state
        self.tilt_lp = LVector3(0, 0, 1)

        # ── Maze ──────────────────────────────────────────────────────────
        self.maze = loader.loadModel("models/maze")
        self.maze.reparentTo(render)

        self.walls = self.maze.find("**/wall_collide")
        self.walls.node().setIntoCollideMask(BitMask32.bit(0))

        self.loseTriggers = []
        for i in range(6):
            trigger = self.maze.find("**/hole_collide" + str(i))
            trigger.node().setIntoCollideMask(BitMask32.bit(0))
            trigger.node().setName("loseTrigger")
            self.loseTriggers.append(trigger)

        self.mazeGround = self.maze.find("**/ground_collide")
        self.mazeGround.node().setIntoCollideMask(BitMask32.bit(1))

        # ── Ball ──────────────────────────────────────────────────────────
        self.ballRoot = render.attachNewNode("ballRoot")
        self.ball = loader.loadModel("models/ball")
        self.ball.reparentTo(self.ballRoot)

        self.ballSphere = self.ball.find("**/ball")
        self.ballSphere.node().setFromCollideMask(BitMask32.bit(0))
        self.ballSphere.node().setIntoCollideMask(BitMask32.allOff())

        self.ballGroundRay = CollisionRay()
        self.ballGroundRay.setOrigin(0, 0, 10)
        self.ballGroundRay.setDirection(0, 0, -1)
        self.ballGroundCol = CollisionNode("groundRay")
        self.ballGroundCol.addSolid(self.ballGroundRay)
        self.ballGroundCol.setFromCollideMask(BitMask32.bit(1))
        self.ballGroundCol.setIntoCollideMask(BitMask32.allOff())
        self.ballGroundColNp = self.ballRoot.attachNewNode(self.ballGroundCol)

        self.cTrav = CollisionTraverser()
        self.cHandler = CollisionHandlerQueue()
        self.cTrav.addCollider(self.ballSphere, self.cHandler)
        self.cTrav.addCollider(self.ballGroundColNp, self.cHandler)

        # ── Lighting ──────────────────────────────────────────────────────
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.55, .55, .55, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(LVector3(0, 0, -1))
        directionalLight.setColor((0.375, 0.375, 0.375, 1))
        directionalLight.setSpecularColor((1, 1, 1, 1))
        self.ballRoot.setLight(render.attachNewNode(ambientLight))
        self.ballRoot.setLight(render.attachNewNode(directionalLight))

        m = Material()
        m.setSpecular((1, 1, 1, 1))
        m.setShininess(96)
        self.ball.setMaterial(m, 1)

        self.start()

    def _set_key(self, key, pressed):
        self.keys[key] = pressed

    def start(self):
        startPos = self.maze.find("**/start").getPos()
        self.ballRoot.setPos(startPos)
        self.ballV = LVector3(0, 0, 0)
        self.accelV = LVector3(0, 0, 0)
        taskMgr.remove("rollTask")
        self.mainLoop = taskMgr.add(self.rollTask, "rollTask")

    def groundCollideHandler(self, colEntry):
        newZ = colEntry.getSurfacePoint(render).getZ()
        self.ballRoot.setZ(newZ + .4)
        norm = colEntry.getSurfaceNormal(render)
        accelSide = norm.cross(LVector3.up())
        self.accelV = norm.cross(accelSide)

    def wallCollideHandler(self, colEntry):
        norm = colEntry.getSurfaceNormal(render) * -1
        curSpeed = self.ballV.length()
        if curSpeed < 1e-6:
            return
        inVec = self.ballV / curSpeed
        velAngle = norm.dot(inVec)
        hitDir = colEntry.getSurfacePoint(render) - self.ballRoot.getPos()
        hitDir.normalize()
        hitAngle = norm.dot(hitDir)
        if velAngle > 0 and hitAngle > .995:
            reflectVec = (norm * norm.dot(inVec * -1) * 2) + inVec
            self.ballV = reflectVec * (curSpeed * (((1 - velAngle) * .5) + .5))
            disp = colEntry.getSurfacePoint(render) - colEntry.getInteriorPoint(render)
            self.ballRoot.setPos(self.ballRoot.getPos() + disp)

    def _keyboard_tilt(self):
        tx, ty = 0.0, 0.0
        if self.keys.get("arrow_left"):
            tx -= 0.3
        if self.keys.get("arrow_right"):
            tx += 0.3
        if self.keys.get("arrow_up"):
            ty += 0.3
        if self.keys.get("arrow_down"):
            ty -= 0.3
        return tx, ty

    def rollTask(self, task):
        dt = base.clock.dt
        if dt > .2:
            return Task.cont

        # ── Collision dispatch ────────────────────────────────────────────
        for i in range(self.cHandler.getNumEntries()):
            entry = self.cHandler.getEntry(i)
            name = entry.getIntoNode().getName()
            if name == "wall_collide":
                self.wallCollideHandler(entry)
            elif name == "ground_collide":
                self.groundCollideHandler(entry)
            elif name == "loseTrigger":
                self.loseGame(entry)

        # ── Tilt input ────────────────────────────────────────────────────
        ax, ay, az = self.sensor.poll()

        alpha = min(1.0, dt * 6.0)
        self.tilt_lp = self.tilt_lp * (1.0 - alpha) + LVector3(ax, ay, az) * alpha

        use_sensor = self.sensor.has_recent_data()
        if use_sensor:
            tx = max(-0.55, min(0.55, self.tilt_lp.x))
            ty = max(-0.55, min(0.55, self.tilt_lp.y))
        else:
            tx, ty = self._keyboard_tilt()

        # Tilt the maze board (roll = left/right, pitch = front/back)
        self.maze.setR(tx * TILT_SCALE)
        self.maze.setP(ty * -TILT_SCALE)

        # ── Physics ───────────────────────────────────────────────────────
        self.ballV += self.accelV * dt * ACCEL
        if self.ballV.lengthSquared() > MAX_SPEED_SQ:
            self.ballV.normalize()
            self.ballV *= MAX_SPEED
        self.ballRoot.setPos(self.ballRoot.getPos() + self.ballV * dt)

        prevRot = LRotationf(self.ball.getQuat())
        axis = LVector3.up().cross(self.ballV)
        newRot = LRotationf(axis, 45.5 * dt * self.ballV.length())
        self.ball.setQuat(prevRot * newRot)

        # ── Status text ───────────────────────────────────────────────────
        if use_sensor:
            self.status.setText(
                f"tilt x={tx:+.3f} y={ty:+.3f}  speed={self.ballV.length():.2f}"
            )
        else:
            self.status.setText("keyboard fallback: arrow keys")

        return Task.cont

    def loseGame(self, entry):
        toPos = entry.getInteriorPoint(render)
        taskMgr.remove("rollTask")
        Sequence(
            Parallel(
                LerpFunc(self.ballRoot.setX, fromData=self.ballRoot.getX(),
                         toData=toPos.getX(), duration=.1),
                LerpFunc(self.ballRoot.setY, fromData=self.ballRoot.getY(),
                         toData=toPos.getY(), duration=.1),
                LerpFunc(self.ballRoot.setZ, fromData=self.ballRoot.getZ(),
                         toData=self.ballRoot.getZ() - .9, duration=.2),
            ),
            Wait(1),
            Func(self.start),
        ).start()

    def _quit(self):
        self.sensor.stop()
        self.userExit()

    def destroy(self):
        self.sensor.stop()
        super().destroy()


def main():
    game = TiltMazeGame()
    game.run()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()

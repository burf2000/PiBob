#!/usr/bin/env python3
"""Dry-run PiBobAssembly.py against a MOCK of the Fusion API (no Fusion needed).

    uv run --with numpy --with trimesh --with pillow fusion/verify/mock_fusion_run.py

This is not Fusion. It implements just the adsk.core / adsk.fusion calls the script
makes, with a tiny joint solver that behaves like a Fusion as-built revolute joint:
driving it rotates the first occurrence about the custom-axis entity (in its
zero-pose place, carried along by upstream joints) through the joint-geometry point.

What it proves:
  * the script runs end to end (no typos / wrong arg counts in OUR code);
  * the cm conversion + occurrence/sub-occurrence placement puts every mesh vertex
    exactly where preview_pose.py (and therefore RViz) puts it;
  * the joint pivots/axes/sign handling reproduce the URDF motion, for BOTH possible
    Fusion sign conventions (the self-check must detect and fix the opposite one);
  * the limits land in Fusion's sign convention.
What it cannot prove: that real Fusion accepts each call (see README "uncertain API").
"""
import importlib.util
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FUSION_DIR = os.path.dirname(HERE)
sys.path.insert(0, FUSION_DIR)
import preview_pose as pp  # noqa: E402

FUSION_POSITIVE = [1]          # +1: Fusion + = right-hand about the axis entity; -1: opposite
MESH_CM = {"mm": 0.1, "cm": 1.0, "m": 100.0, "in": 2.54}


# ---------------------------------------------------------------- adsk.core
class Matrix3D:
    def __init__(self):
        self.m = np.eye(4)

    @staticmethod
    def create():
        return Matrix3D()

    def setCell(self, r, c, v):
        self.m[r, c] = v

    def setToRotation(self, angle, axis, origin):
        self.m = pp.rot_about_line(origin.p, axis.v, angle)
        return True

    def copy(self):
        n = Matrix3D()
        n.m = self.m.copy()
        return n


class Point3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.p = np.array([x, y, z], float)

    @staticmethod
    def create(x=0.0, y=0.0, z=0.0):
        return Point3D(x, y, z)

    def copy(self):
        return Point3D(*self.p)

    def transformBy(self, M):
        self.p = M.m[:3, :3] @ self.p + M.m[:3, 3]
        return True

    def distanceTo(self, o):
        return float(np.linalg.norm(self.p - o.p))


class Vector3D:
    def __init__(self, x, y, z):
        self.v = np.array([x, y, z], float)

    @staticmethod
    def create(x, y, z):
        return Vector3D(x, y, z)


class UI:
    def __init__(self):
        self.messages = []

    def messageBox(self, text, title=""):
        self.messages.append(text)

    def createFileDialog(self):
        raise AssertionError("file dialog should not be needed")


class App:
    _inst = None

    def __init__(self):
        self.userInterface = UI()
        self.activeProduct = None
        self.documents = types.SimpleNamespace(add=self._add_doc)

    def _add_doc(self, typ):
        self.activeProduct = Design()
        return object()

    @staticmethod
    def get():
        if App._inst is None:
            App._inst = App()
        return App._inst


core = types.ModuleType("adsk.core")
core.Matrix3D, core.Point3D, core.Vector3D, core.Application = Matrix3D, Point3D, Vector3D, App
core.DocumentTypes = types.SimpleNamespace(FusionDesignDocumentType=1)
core.DialogResults = types.SimpleNamespace(DialogOK=0)


# ---------------------------------------------------------------- adsk.fusion
class Entity:
    """Construction point/axis/sketch line of a component: local point + local direction."""

    def __init__(self, comp, point, direction=None):
        self.comp, self.point, self.direction = comp, np.asarray(point, float), direction

    def createForAssemblyContext(self, occ):
        assert occ.component is self.comp, "proxy through the wrong occurrence"
        return types.SimpleNamespace(entity=self, occ=occ)


class Sketch:
    def __init__(self, comp):
        self.comp, self.name = comp, ""
        self.sketchCurves = types.SimpleNamespace(sketchLines=types.SimpleNamespace(addByTwoPoints=self._line))

    def modelToSketchSpace(self, p):
        return p

    def _line(self, a, b):
        d = b.p - a.p
        e = Entity(self.comp, a.p, d / np.linalg.norm(d))
        e.isConstruction = False
        return e


class BaseFeature:
    def __init__(self):
        self.editing = False

    def startEdit(self):
        self.editing = True

    def finishEdit(self):
        self.editing = False


class Component:
    def __init__(self, design):
        self.name, self.design = "", design
        self.occurrences = Occurrences(self, design)
        self.originConstructionPoint = Entity(self, [0, 0, 0])
        self.xConstructionAxis = Entity(self, [0, 0, 0], np.array([1.0, 0, 0]))
        self.yConstructionAxis = Entity(self, [0, 0, 0], np.array([0, 1.0, 0]))
        self.zConstructionAxis = Entity(self, [0, 0, 0], np.array([0, 0, 1.0]))
        self.xYConstructionPlane = object()
        self.meshes = []
        self.features = types.SimpleNamespace(baseFeatures=types.SimpleNamespace(add=BaseFeature))
        self.meshBodies = types.SimpleNamespace(add=self._add_mesh)
        self.sketches = types.SimpleNamespace(add=lambda plane: Sketch(self))
        self.asBuiltJoints = AsBuiltJoints(design)

    def _add_mesh(self, path, unit, bf):
        assert isinstance(bf, BaseFeature) and bf.editing, "mesh import must be inside an editing base feature"
        assert os.path.isfile(path), path
        self.meshes.append((path, unit))
        body = types.SimpleNamespace(name="")
        return types.SimpleNamespace(count=1, item=lambda i: body)


class Occurrence:
    def __init__(self, comp, local, parent_comp):
        self.component, self.local, self.parent_comp = comp, local.m.copy(), parent_comp
        self.zero = local.m.copy()
        self.isGrounded = False
        self.isGroundToParent = False

    @property
    def transform2(self):
        M = Matrix3D()
        M.m = self.local.copy()
        return M


class Occurrences:
    def __init__(self, comp, design):
        self.comp, self.design, self.items = comp, design, []

    def addNewComponent(self, matrix):
        occ = Occurrence(Component(self.design), matrix, self.comp)
        self.items.append(occ)
        return occ


class RotationLimits:
    isMinimumValueEnabled = isMaximumValueEnabled = isRestValueEnabled = False
    minimumValue = maximumValue = restValue = 0.0


class JointMotion:
    def __init__(self, design):
        self.design, self._v = design, 0.0
        self.rotationLimits = RotationLimits()

    @property
    def rotationValue(self):
        return self._v

    @rotationValue.setter
    def rotationValue(self, v):
        self._v = v
        self.design.solve()


class JointInput:
    def __init__(self, o1, o2, geo):
        self.o1, self.o2, self.geo, self.kind, self.axis = o1, o2, geo, None, None

    def setAsRevoluteJointMotion(self, direction, entity):
        assert direction == "custom" and entity is not None
        self.kind, self.axis = "revolute", entity

    def setAsRigidJointMotion(self):
        self.kind = "rigid"


class AsBuiltJoints:
    def __init__(self, design):
        self.design = design

    def createInput(self, o1, o2, geo):
        return JointInput(o1, o2, geo)

    def add(self, inp):
        assert inp.kind, "joint motion not set"
        if inp.kind == "revolute":
            assert inp.geo is not None
        j = types.SimpleNamespace(inp=inp, name="", jointMotion=JointMotion(self.design))
        self.design.joints.append(j)
        return j


class Design:
    def __init__(self):
        self.designType = None
        self.joints = []
        self.fusionUnitsManager = types.SimpleNamespace(distanceDisplayUnits=None)
        self.rootComponent = Component(self)

    def solve(self):
        """Propagate joints from grounded occurrences (Fusion-like as-built semantics)."""
        occs = self.rootComponent.occurrences.items
        known = {id(o) for o in occs if o.isGrounded}
        for o in occs:
            if o.isGrounded:
                o.local = o.zero.copy()
        pending = list(self.joints)
        while pending:
            progressed = False
            for j in list(pending):
                o1, o2 = j.inp.o1, j.inp.o2
                if id(o2) not in known:
                    continue
                D = o2.local @ np.linalg.inv(o2.zero)
                if j.inp.kind == "revolute":
                    ent = j.inp.axis.entity
                    Z = j.inp.axis.occ.zero
                    piv = Z[:3, :3] @ j.inp.geo.entity.point + Z[:3, 3]
                    ax = Z[:3, :3] @ ent.direction
                    R = pp.rot_about_line(piv, ax, FUSION_POSITIVE[0] * j.jointMotion.rotationValue)
                else:
                    R = np.eye(4)
                o1.local = D @ R @ o1.zero
                known.add(id(o1))
                pending.remove(j)
                progressed = True
            assert progressed, "joint graph not connected to ground"


class JointGeometry:
    @staticmethod
    def createByPoint(p):
        return p


fusion = types.ModuleType("adsk.fusion")
fusion.Design = types.SimpleNamespace(cast=lambda x: x)
fusion.DesignTypes = types.SimpleNamespace(ParametricDesignType=1)
fusion.DistanceUnits = types.SimpleNamespace(MillimeterDistanceUnits=1)
fusion.MeshUnits = types.SimpleNamespace(MillimeterMeshUnit="mm", CentimeterMeshUnit="cm",
                                         MeterMeshUnit="m", InchMeshUnit="in")
fusion.JointDirections = types.SimpleNamespace(CustomJointDirection="custom")
fusion.JointGeometry = JointGeometry

adsk = types.ModuleType("adsk")
adsk.core, adsk.fusion, adsk.doEvents = core, fusion, lambda: None
sys.modules.update({"adsk": adsk, "adsk.core": core, "adsk.fusion": fusion})


# ---------------------------------------------------------------- run + compare
def load_script():
    spec = importlib.util.spec_from_file_location("PiBobAssembly",
                                                  os.path.join(FUSION_DIR, "PiBobAssembly", "PiBobAssembly.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def fusion_vertices(design):
    """{link: [world verts in METRES]} from the mock's occurrence tree + imported meshes."""
    import trimesh
    out = {}
    for occ in design.rootComponent.occurrences.items:
        for v in occ.component.occurrences.items:
            for path, unit in v.component.meshes:
                verts = np.asarray(trimesh.load(path, force="mesh", process=False).vertices) * MESH_CM[unit]
                M = occ.local @ v.local
                out.setdefault(occ.component.name, []).append(((M[:3, :3] @ verts.T).T + M[:3, 3]) / 100.0)
    return out


def compare(data, design, angles):
    W = pp.pose_world(data, angles)
    ref = {}
    for link, v, f, _ in pp.posed_meshes(data, W):
        ref.setdefault(link, []).append(v)
    got = fusion_vertices(design)
    err = 0.0
    for link, lst in ref.items():
        assert len(lst) == len(got[link]), link
        for a, b in zip(lst, got[link]):
            err = max(err, float(np.abs(a - b).max()))
    return err * 1000  # mm


def run_once(sign):
    FUSION_POSITIVE[0] = sign
    App._inst = None
    mod = load_script()
    mod.run(None)
    app = App.get()
    design = app.activeProduct
    msg = app.userInterface.messages[-1]
    data = pp.load()
    print(f"--- mock Fusion with positive rotation = {'right-hand' if sign > 0 else 'LEFT-hand'} ---")
    print(msg)
    assert "failed" not in msg, msg
    ok = msg.count("OK ")
    nrev = sum(j["type"] == "revolute" for j in data["joints"])
    assert ok == nrev, f"only {ok}/{nrev} joints self-checked OK"

    zero_err = compare(data, design, {})
    print(f"zero pose: max vertex error vs preview_pose = {zero_err:.6f} mm")

    # Drive a combined pose through the Fusion joints (in each joint's Fusion sign) and compare.
    pose = {"head_pan_joint": 0.6, "head_tilt_joint": -0.4, "r_shoulder_rot_joint": 0.8,
            "r_shoulder_lift_joint": 0.3, "r_arm_rot_joint": -0.6, "r_bicep_joint": 1.2,
            "l_shoulder_rot_joint": -0.3, "l_shoulder_lift_joint": -0.3, "l_arm_rot_joint": 0.7,
            "l_bicep_joint": 0.9}
    spec = {j["name"]: j for j in data["joints"]}
    for jt in design.joints:
        if jt.inp.kind != "revolute":
            continue
        s = spec[jt.name]
        lim = jt.jointMotion.rotationLimits
        flipped = abs(lim.maximumValue + s["lower"]) < 1e-9 and abs(lim.minimumValue + s["upper"]) < 1e-9
        straight = abs(lim.maximumValue - s["upper"]) < 1e-9 and abs(lim.minimumValue - s["lower"]) < 1e-9
        assert flipped or straight, f"{jt.name}: limits {lim.minimumValue},{lim.maximumValue} match neither sign"
        assert lim.isMinimumValueEnabled and lim.isMaximumValueEnabled and lim.restValue == 0.0
        v = pose[jt.name] * mod.SIGNS[jt.name]
        assert (flipped if mod.SIGNS[jt.name] < 0 else straight), f"{jt.name}: limits disagree with sign"
        assert lim.minimumValue - 1e-9 <= v <= lim.maximumValue + 1e-9, f"{jt.name} test value outside limits"
        jt.jointMotion.rotationValue = v
    pose_err = compare(data, design, pose)
    print(f"combined pose: max vertex error vs preview_pose = {pose_err:.6f} mm")
    assert zero_err < 1e-6 and pose_err < 1e-6
    names = [o.component.name for o in design.rootComponent.occurrences.items]
    grounded = [o.component.name for o in design.rootComponent.occurrences.items if o.isGrounded]
    print(f"{len(names)} link components, grounded: {grounded}, {len(design.joints)} joints")
    return zero_err, pose_err


if __name__ == "__main__":
    run_once(+1)
    run_once(-1)
    print("\nMOCK RUN PASSED (both Fusion sign conventions)")

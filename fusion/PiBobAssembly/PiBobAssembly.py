"""PiBob URDF -> Fusion 360 assembly.

Fusion: Utilities > ADD-INS > Scripts and Add-Ins > "+" (next to My Scripts) > pick this
PiBobAssembly folder > select PiBobAssembly > Run.

Reads ../pibob_fusion.json (made by fusion/urdf_to_fusion_data.py) and builds, in a NEW
parametric design:

  * one root-level component per URDF link, its occurrence placed at the link's
    zero-pose world transform (so the link component's ORIGIN == the URDF link frame
    == the joint frame of the joint that drives it);
  * inside each link component, one sub-component per URDF <visual>, placed at the
    visual <origin> and holding the STL as a mesh body (imported in mm inside a
    Base Feature, as a parametric design requires). The sub-component frame is the
    STL's own native frame, so a STEP/f3d export of the same part drops straight in;
  * one AS-BUILT joint per URDF joint (nothing jumps): revolute about the URDF axis
    through the URDF pivot, with the URDF min/max limits (no rest value - that makes
    joints snap back); rigid for fixed joints. Each revolute joint's geometry is a
    construction sketch line in the child LINK component, from its origin (the pivot)
    along the URDF axis: JointGeometry.createByCurve(line, StartKeyPoint) +
    ZAxisJointDirection. Never mesh/BRep geometry, so deleting or replacing a part's
    body leaves every joint intact;
  * base_link grounded.

After building, the script drives every revolute joint to a test angle, measures
the child's actual rotation axis/angle and pivot drift from its world transform,
compares with the URDF axis (flags > 5 deg), fixes the sign convention if Fusion's
positive direction is opposite, puts the joint back to 0 and reports. The report is also written to fusion_check.txt next to
the JSON.

UNITS: the Fusion API works in CENTIMETRES (Matrix3D translations, Point3D) and
radians. The JSON is metres -> multiply lengths by 100 (CM below). Mesh import
units are separate: the STLs are millimetres (URDF scale 0.001) -> MillimeterMeshUnit.
"""
import json
import math
import os
import traceback

import adsk.core
import adsk.fusion

CM = 100.0                     # metres -> Fusion internal centimetres
JSON_NAME = "pibob_fusion.json"
TEST_ANGLE = 0.3               # rad, used by the post-build joint self-check
TOL_CM = 0.01                  # 0.1 mm
AXIS_TOL_DEG = 5.0             # self-check flags a joint whose real axis is further off than this
AXIS_LINE_CM = 2.0             # length of the construction axis line drawn in each child link

# joint name -> +1 if Fusion's positive angle == the URDF's positive angle, -1 if opposite.
# Filled by run() from the measured self-check. The axis line is drawn along the URDF
# axis INCLUDING its sign, so +1 is expected everywhere; -1 would mean Fusion's joint Z
# runs end->start of the line, and the limits are flipped to match.
SIGNS = {}

# URDF mesh scale -> Fusion mesh import unit (the STL numbers are in this unit).
MESH_UNITS = {
    0.001: adsk.fusion.MeshUnits.MillimeterMeshUnit,
    0.01: adsk.fusion.MeshUnits.CentimeterMeshUnit,
    1.0: adsk.fusion.MeshUnits.MeterMeshUnit,
    0.0254: adsk.fusion.MeshUnits.InchMeshUnit,
}


# ---------------------------------------------------------------- helpers
def matrix_from_json(T):
    """JSON 4x4 (metres, row-major nested list) -> Matrix3D (cm).

    setCell(row, col, value) is used instead of setWithArray so there is no
    row- vs column-major ambiguity."""
    m = adsk.core.Matrix3D.create()
    for r in range(4):
        for c in range(4):
            v = T[r][c]
            if c == 3 and r < 3:
                v *= CM
            m.setCell(r, c, v)
    return m


def occurrence_world(occ):
    """World transform of a root-level occurrence (transform2 is the current API; transform is the older name)."""
    try:
        return occ.transform2
    except AttributeError:
        return occ.transform


def find_json(ui):
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.join(here, JSON_NAME), os.path.join(here, "..", JSON_NAME)):
        if os.path.isfile(cand):
            return os.path.normpath(cand)
    dlg = ui.createFileDialog()
    dlg.title = "Can't find {} next to the script - pick it".format(JSON_NAME)
    dlg.filter = "PiBob Fusion data (*.json)"
    if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.filename


def resolve_stl(json_dir, vis):
    """The STL path is stored relative to the JSON; also accept a flat copy next to it."""
    for cand in (os.path.join(json_dir, vis["stl"]),
                 os.path.join(json_dir, "meshes", os.path.basename(vis["stl"])),
                 os.path.join(json_dir, os.path.basename(vis["stl"]))):
        if os.path.isfile(cand):
            return os.path.normpath(cand)
    return None


def mesh_unit(scale):
    if abs(scale[0] - scale[1]) > 1e-12 or abs(scale[0] - scale[2]) > 1e-12:
        raise ValueError("non-uniform mesh scale {} is not supported".format(scale))
    for s, unit in MESH_UNITS.items():
        if abs(scale[0] - s) < 1e-9:
            return unit
    raise ValueError("mesh scale {} has no matching Fusion mesh unit".format(scale))


# ---------------------------------------------------------------- build
def build_links(root, data, json_dir):
    """One root-level component per link + one sub-component per visual. Returns {link: occurrence}."""
    occs = {}
    for link in data["links"]:
        occ = root.occurrences.addNewComponent(matrix_from_json(link["world"]))
        comp = occ.component
        comp.name = link["name"]
        occs[link["name"]] = occ

        for i, vis in enumerate(link["visuals"]):
            stl = resolve_stl(json_dir, vis)
            part = os.path.splitext(os.path.basename(vis["stl"]))[0]
            # Sub-occurrence transform is RELATIVE TO the link component = URDF <visual><origin>.
            # (We place the mesh with an occurrence transform rather than moving the mesh
            # body, because occurrence transforms are core, well-trodden API; moving a
            # MeshBody with a Move feature is not something we can test here.)
            vocc = comp.occurrences.addNewComponent(matrix_from_json(vis["origin"]))
            vcomp = vocc.component
            vcomp.name = "{}__{}".format(link["name"], part) + ("" if i == 0 else "_{}".format(i))
            # Keep the part locked to its link (it still moves with the link's joint).
            try:
                vocc.isGroundToParent = True
            except Exception:
                pass

            # Parametric designs need mesh import inside a Base Feature.
            bf = vcomp.features.baseFeatures.add()
            bf.startEdit()
            try:
                bodies = vcomp.meshBodies.add(stl, mesh_unit(vis["scale"]), bf)
            finally:
                bf.finishEdit()
            try:
                for k in range(bodies.count):
                    bodies.item(k).name = part
            except Exception:
                pass
    return occs


def axis_line_for(child_occ, axis_local, joint_name):
    """Construction sketch line in the CHILD link component: from its origin (= the URDF
    pivot) 2 cm along the URDF axis, SIGN INCLUDED (a -Y axis is drawn towards -Y).

    Returned as a proxy in the root (assembly) context. The joint is then built with
    JointGeometry.createByCurve(line, StartKeyPoint) + ZAxisJointDirection: for a straight
    curve the joint origin is the start point and the joint's primary (Z) axis runs
    along the line, so the rotation axis is explicit geometry rather than a
    CustomJointDirection entity (which real Fusion ignored - see README troubleshooting).
    Sketch geometry is not mesh/BRep, so swapping a part body leaves it intact."""
    comp = child_occ.component
    ax = [float(v) for v in axis_local]
    # Put the sketch on a construction plane that CONTAINS the axis so the line is planar.
    if abs(ax[2]) < 1e-9:
        plane = comp.xYConstructionPlane
    elif abs(ax[1]) < 1e-9:
        plane = comp.xZConstructionPlane
    elif abs(ax[0]) < 1e-9:
        plane = comp.yZConstructionPlane
    else:
        plane = comp.xYConstructionPlane      # general axis: a 3D sketch line (untested path)
    sk = comp.sketches.add(plane)
    sk.name = "{}_axis".format(joint_name)
    # modelToSketchSpace takes the component's own coordinates (the sketch is not a proxy),
    # which also absorbs the XZ-plane sketch's flipped Y.
    p0 = sk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, 0))
    p1 = sk.modelToSketchSpace(adsk.core.Point3D.create(*[AXIS_LINE_CM * v for v in ax]))
    line = sk.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
    line.isConstruction = True
    return line.createForAssemblyContext(child_occ)


def principal_axis(axis, tol=1e-6):
    """(index 0/1/2, sign +1/-1) if axis is +-X/Y/Z, else None."""
    for i in range(3):
        if abs(abs(axis[i]) - 1.0) < tol and all(abs(axis[k]) < tol for k in range(3) if k != i):
            return i, (1 if axis[i] > 0 else -1)
    return None


# Ways of building a revolute joint, tried in order until the self-check passes:
#   0  "axis-line": createByCurve(axis sketch line, StartKeyPoint) + ZAxisJointDirection
#   1  "point+XYZ": createByPoint(child origin point) + X/Y/ZAxisJointDirection
#      (only for axes along the link's X/Y/Z; relies on a point's joint frame being the
#      component's axes - true for every PiBob link, which are all world-aligned)
STRATEGIES = ("axis-line", "point+XYZ")


def make_revolute(root, occs, j, strategy):
    """Create one as-built revolute joint for URDF joint j. Returns the joint or None
    if the strategy doesn't apply to this axis."""
    child, parent = occs[j["child"]], occs[j["parent"]]
    if strategy == 0:
        line = axis_line_for(child, j["axis_local"], j["name"])
        geo = adsk.fusion.JointGeometry.createByCurve(line, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
        direction = adsk.fusion.JointDirections.ZAxisJointDirection
    else:
        pa = principal_axis(j["axis_local"])
        if pa is None:
            return None
        pivot = child.component.originConstructionPoint.createForAssemblyContext(child)
        geo = adsk.fusion.JointGeometry.createByPoint(pivot)
        direction = (adsk.fusion.JointDirections.XAxisJointDirection,
                     adsk.fusion.JointDirections.YAxisJointDirection,
                     adsk.fusion.JointDirections.ZAxisJointDirection)[pa[0]]
    inp = root.asBuiltJoints.createInput(child, parent, geo)
    inp.setAsRevoluteJointMotion(direction)
    joint = root.asBuiltJoints.add(inp)
    joint.name = j["name"]
    return joint


def build_joints(root, data, occs):
    """As-built joints (revolute ones with strategy 0). Returns [[joint, spec]] for the revolute ones."""
    revs = []
    for j in data["joints"]:
        child, parent = occs[j["child"]], occs[j["parent"]]
        if j["type"] in ("revolute", "continuous"):
            revs.append([make_revolute(root, occs, j, 0), j])
        else:
            if j["type"] != "fixed":
                print("joint {} type {} -> rigid".format(j["name"], j["type"]))
            try:
                inp = root.asBuiltJoints.createInput(child, parent, None)   # rigid: geometry optional
                inp.setAsRigidJointMotion()
                joint = root.asBuiltJoints.add(inp)
            except Exception:
                # Pivot = the child link component's origin (URDF: child frame == joint frame).
                pivot = child.component.originConstructionPoint.createForAssemblyContext(child)
                inp = root.asBuiltJoints.createInput(child, parent, adsk.fusion.JointGeometry.createByPoint(pivot))
                inp.setAsRigidJointMotion()
                joint = root.asBuiltJoints.add(inp)
            joint.name = j["name"]
    return revs


def check_and_repair(root, occs, data, entry):
    """Self-check the joint; if its real axis is wrong, rebuild it with the next strategy.
    entry = [joint, spec] (joint replaced in place). Returns (sign, ok, message)."""
    joint, j = entry
    tried, first = [], None
    for strategy in range(len(STRATEGIES)):
        if strategy > 0:
            new = make_revolute(root, occs, j, strategy)
            if new is None:
                continue
            joint.deleteMe()            # delete the failed one only once a replacement exists
            joint = new
            entry[0] = joint
        try:
            sign, ok, msg = self_check(occs, data, joint, j)
        except Exception as e:
            sign, ok, msg = 1, False, "UNVERIFIED (self-check failed: {})".format(e)
        tried.append(STRATEGIES[strategy])
        if ok:
            return sign, True, "{} [{}]".format(msg, STRATEGIES[strategy])
        if first is None:
            first = (sign, msg)
    # Nothing passed: go back to strategy 0 and report its result.
    if len(tried) > 1:
        new = make_revolute(root, occs, j, 0)
        entry[0].deleteMe()
        entry[0] = new
    sign, msg = first
    return sign, False, "{} [tried {}]".format(msg, ", ".join(tried))


def set_limits(joint, j, sign):
    """URDF limits in the Fusion joint's own positive direction. NO rest value: an
    enabled rest value makes every joint spring back to it after a drag."""
    if j["type"] != "revolute":
        return
    lo, hi = (j["lower"], j["upper"]) if sign > 0 else (-j["upper"], -j["lower"])
    lim = joint.jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = lo
    lim.isMaximumValueEnabled = True
    lim.maximumValue = hi
    lim.isRestValueEnabled = False


def _rot3(M):
    return [[M.getCell(r, c) for c in range(3)] for r in range(3)]


def rotation_axis_angle(A, B):
    """Axis (unit list) + angle (rad, 0..pi) of the rotation taking frame A to frame B
    (Matrix3D world transforms): R = B.rot * A.rot^T."""
    a, b = _rot3(A), _rot3(B)
    R = [[sum(b[r][k] * a[c][k] for k in range(3)) for c in range(3)] for r in range(3)]
    tr = R[0][0] + R[1][1] + R[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-12:
        return None, ang
    return [x / n for x in v], ang


def self_check(occs, data, joint, j):
    """Drive the joint to TEST_ANGLE and measure what the child ACTUALLY did.

    Reads the child occurrence's world transform before/after and extracts the real
    rotation axis + angle, then compares with the URDF: axis error (deg, flagged > 5),
    pivot drift (mm), and whether Fusion's + is the URDF's + (sign). Returns
    (sign, ok, message). If the transform does not change at all, the API did not
    report the move; the joint may still be fine - check it with Drive Joints."""
    child = occs[j["child"]]
    pivot = adsk.core.Point3D.create(*[v * CM for v in j["origin_world"]])
    motion = joint.jointMotion
    before = occurrence_world(child).copy()
    try:
        motion.rotationValue = TEST_ANGLE
        adsk.doEvents()
        after = occurrence_world(child).copy()
    finally:
        motion.rotationValue = 0.0
        adsk.doEvents()

    axis, ang = rotation_axis_angle(before, after)
    if axis is None or ang < 0.01:
        return 1, False, ("NO UPDATE - transform2 didn't change when the joint was driven; "
                          "can't self-check, use Drive Joints to eyeball it")
    urdf = j["axis_world"]
    dot = sum(axis[k] * urdf[k] for k in range(3))
    sign = 1 if dot >= 0 else -1
    axis_err = math.degrees(math.acos(min(1.0, abs(dot))))
    # pivot drift: where the zero-pose pivot ends up after the move
    p = pivot.copy()
    inv = before.copy()
    inv.invert()
    p.transformBy(inv)
    p.transformBy(after)
    drift_mm = p.distanceTo(pivot) * 10
    ok = axis_err <= AXIS_TOL_DEG and drift_mm <= TOL_CM * 10 and abs(ang - TEST_ANGLE) < 0.01
    msg = "{}  axis err {:.2f} deg, pivot drift {:.3f} mm, turned {:.3f} rad{}".format(
        "OK   " if ok else "WRONG", axis_err, drift_mm, ang,
        "" if sign > 0 else " (Fusion + = URDF -, limits flipped)")
    return sign, ok, msg


# ---------------------------------------------------------------- entry point
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        json_path = find_json(ui)
        if not json_path:
            ui.messageBox("No {} selected - nothing built.\n\nGenerate it with:\n"
                          "  uv run --with xacro --with numpy fusion/urdf_to_fusion_data.py".format(JSON_NAME),
                          "PiBob assembly")
            return
        with open(json_path) as f:
            data = json.load(f)
        json_dir = os.path.dirname(json_path)

        missing = [v["stl"] for l in data["links"] for v in l["visuals"] if resolve_stl(json_dir, v) is None]
        if missing:
            ui.messageBox("These STL files were not found (paths are relative to\n{}):\n\n{}\n\n"
                          "Keep the fusion/ folder inside the PiBob repo checkout, or copy the STLs "
                          "into fusion/meshes/.".format(json_dir, "\n".join(missing)), "PiBob assembly")
            return

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        try:
            design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        except Exception:
            pass
        root = design.rootComponent

        occs = build_links(root, data, json_dir)
        occs[data["root_link"]].isGrounded = True
        revs = build_joints(root, data, occs)

        report, bad = [], 0
        for entry in revs:
            j = entry[1]
            sign, ok, msg = check_and_repair(root, occs, data, entry)
            set_limits(entry[0], j, sign)
            SIGNS[j["name"]] = sign
            bad += 0 if ok else 1
            report.append("{:24s} {}".format(j["name"], msg))

        text = ("PiBob built: {} links, {} joints ({} revolute).\n\n"
                "Joint self-check (drive to {:.2f} rad, measure the real axis vs the URDF):\n{}\n\n{}"
                "File > Save as to keep it (.f3d via File > Export).").format(
            len(data["links"]), len(data["joints"]), len(revs), TEST_ANGLE, "\n".join(report),
            "All joints OK.\n\n" if bad == 0 else
            "{} joint(s) need a look - see fusion/README.md troubleshooting.\n\n".format(bad))
        try:
            with open(os.path.join(json_dir, "fusion_check.txt"), "w") as f:
                f.write(text + "\n")
        except Exception:
            pass
        ui.messageBox(text, "PiBob assembly")
    except Exception:
        if ui:
            ui.messageBox("PiBob assembly failed:\n{}".format(traceback.format_exc()), "PiBob assembly")

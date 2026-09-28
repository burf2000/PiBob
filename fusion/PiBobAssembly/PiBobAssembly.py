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
    through the URDF pivot, with the URDF limits; rigid for fixed joints. Joint
    geometry uses ONLY construction geometry of the child LINK component (its origin
    point + a construction axis), never mesh/BRep geometry, so deleting or replacing
    a part's body leaves every joint intact;
  * base_link grounded.

After building, the script drives every revolute joint to a test angle, checks the
child moved exactly as the URDF says (pivot fixed, test point where expected),
fixes the sign convention if Fusion's positive direction is opposite, puts the
joint back to 0 and reports. The report is also written to fusion_check.txt next to
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

# joint name -> +1 if Fusion's positive angle == the URDF's positive angle, -1 if opposite.
# (Filled by run(); for a -X/-Y/-Z URDF axis the joint is built on the +axis, so the
# Fusion angle is the URDF angle negated unless Fusion's own convention flips it back.)
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


def principal_axis(axis, tol=1e-6):
    """(index 0/1/2, sign +1/-1) if axis is +-X/Y/Z, else None."""
    for i in range(3):
        if abs(abs(axis[i]) - 1.0) < tol and all(abs(axis[k]) < tol for k in range(3) if k != i):
            return i, (1 if axis[i] > 0 else -1)
    return None


def perpendicular(a):
    """Some unit vector perpendicular to a (for the self-check test point)."""
    ref = [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0]
    p = [a[1] * ref[2] - a[2] * ref[1], a[2] * ref[0] - a[0] * ref[2], a[0] * ref[1] - a[1] * ref[0]]
    n = math.sqrt(sum(x * x for x in p))
    return [x / n for x in p]


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


def axis_entity_for(child_occ, axis_local, joint_name):
    """Rotation-axis entity in the CHILD link component, proxied into the root, + sign.

    Returns (entity, sign) where rotating +theta about the entity's positive direction
    equals rotating sign*theta about the URDF axis. Construction geometry only."""
    comp = child_occ.component
    pa = principal_axis(axis_local)
    if pa is not None:
        idx, sign = pa
        axis = (comp.xConstructionAxis, comp.yConstructionAxis, comp.zConstructionAxis)[idx]
        return axis.createForAssemblyContext(child_occ), sign
    # Non-principal axis (not used by PiBob today): a 1 cm construction sketch line from
    # the component origin along the axis. Sketch geometry is not mesh/BRep, so it also
    # survives swapping the part body. UNTESTED PATH - see README.
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "{}_axis".format(joint_name)
    p0 = sk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, 0))
    p1 = sk.modelToSketchSpace(adsk.core.Point3D.create(*axis_local))
    line = sk.sketchCurves.sketchLines.addByTwoPoints(p0, p1)
    line.isConstruction = True
    return line.createForAssemblyContext(child_occ), 1


def build_joints(root, data, occs):
    """As-built joints. Returns [(joint, spec, sign)] for the revolute ones."""
    revs = []
    for j in data["joints"]:
        child, parent = occs[j["child"]], occs[j["parent"]]
        # Pivot = the child link component's origin (URDF: child frame == joint frame).
        pivot = child.component.originConstructionPoint.createForAssemblyContext(child)
        geo = adsk.fusion.JointGeometry.createByPoint(pivot)

        if j["type"] in ("revolute", "continuous"):
            inp = root.asBuiltJoints.createInput(child, parent, geo)
            entity, sign = axis_entity_for(child, j["axis_local"], j["name"])
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, entity)
            joint = root.asBuiltJoints.add(inp)
            joint.name = j["name"]
            revs.append((joint, j, sign))
        else:
            if j["type"] != "fixed":
                print("joint {} type {} -> rigid".format(j["name"], j["type"]))
            try:
                inp = root.asBuiltJoints.createInput(child, parent, None)   # rigid: geometry optional
                inp.setAsRigidJointMotion()
                joint = root.asBuiltJoints.add(inp)
            except Exception:
                inp = root.asBuiltJoints.createInput(child, parent, geo)
                inp.setAsRigidJointMotion()
                joint = root.asBuiltJoints.add(inp)
            joint.name = j["name"]
    return revs


def set_limits(joint, j, sign):
    """URDF limits in the Fusion joint's own positive direction."""
    if j["type"] != "revolute":
        return
    lo, hi = (j["lower"], j["upper"]) if sign > 0 else (-j["upper"], -j["lower"])
    lim = joint.jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = lo
    lim.isMaximumValueEnabled = True
    lim.maximumValue = hi
    lim.isRestValueEnabled = True
    lim.restValue = 0.0


def self_check(occs, data, joint, j, sign):
    """Drive the joint to TEST_ANGLE and compare the child's motion with the URDF.

    Returns (sign_to_use, message). Expected motion: rotate the child's zero-pose
    frame by (sign * TEST_ANGLE) about the URDF world axis through the URDF world pivot.
    If Fusion instead moved it by the opposite angle, its positive direction is the
    other way round for this joint -> flip the sign (limits follow)."""
    child = occs[j["child"]]
    link = next(l for l in data["links"] if l["name"] == j["child"])
    T0 = matrix_from_json(link["world"])
    pivot = adsk.core.Point3D.create(*[v * CM for v in j["origin_world"]])
    axis = adsk.core.Vector3D.create(*j["axis_world"])
    perp = perpendicular(j["axis_local"])
    local = adsk.core.Point3D.create(*[10.0 * v for v in perp])     # 10 cm from the pivot, in the child frame

    def expected(theta_urdf):
        R = adsk.core.Matrix3D.create()
        R.setToRotation(theta_urdf, axis, pivot)
        p = local.copy()
        p.transformBy(T0)
        p.transformBy(R)
        return p

    motion = joint.jointMotion
    try:
        motion.rotationValue = TEST_ANGLE
        adsk.doEvents()
        M = occurrence_world(child)
        got = local.copy()
        got.transformBy(M)
        org = adsk.core.Point3D.create(0, 0, 0)
        org.transformBy(M)
        pivot_err = org.distanceTo(pivot)
        e_same = got.distanceTo(expected(sign * TEST_ANGLE))
        e_flip = got.distanceTo(expected(-sign * TEST_ANGLE))
    finally:
        motion.rotationValue = 0.0
        adsk.doEvents()

    if pivot_err < TOL_CM and e_same < TOL_CM:
        return sign, "OK   pivot err {:.3f} mm, motion err {:.3f} mm".format(pivot_err * 10, e_same * 10)
    if pivot_err < TOL_CM and e_flip < TOL_CM:
        return -sign, "OK   pivot err {:.3f} mm, motion err {:.3f} mm (sign corrected)".format(
            pivot_err * 10, e_flip * 10)
    return sign, ("CHECK pivot err {:.2f} mm, motion err {:.2f} / {:.2f} mm (same/flipped) - "
                  "joint did not move as the URDF says".format(pivot_err * 10, e_same * 10, e_flip * 10))


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

        report = []
        for joint, j, sign in revs:
            try:
                sign, msg = self_check(occs, data, joint, j, sign)
            except Exception as e:
                msg = "UNVERIFIED (self-check failed: {})".format(e)
            set_limits(joint, j, sign)
            SIGNS[j["name"]] = sign
            report.append("{:24s} {}  [Fusion + = URDF {}]".format(j["name"], msg, "+" if sign > 0 else "-"))

        text = ("PiBob built: {} links, {} joints ({} revolute).\n\n"
                "Joint self-check (drive to {:.2f} rad, compare with URDF):\n{}\n\n"
                "File > Save as to keep it (.f3d via File > Export).").format(
            len(data["links"]), len(data["joints"]), len(revs), TEST_ANGLE, "\n".join(report))
        try:
            with open(os.path.join(json_dir, "fusion_check.txt"), "w") as f:
                f.write(text + "\n")
        except Exception:
            pass
        ui.messageBox(text, "PiBob assembly")
    except Exception:
        if ui:
            ui.messageBox("PiBob assembly failed:\n{}".format(traceback.format_exc()), "PiBob assembly")

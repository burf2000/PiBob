# PiBob → Fusion 360 assembly

Opens PiBob in Fusion 360 as a jointed assembly generated from the URDF
(`ros2/pibob_description/urdf/pibob.urdf.xacro`). Each joint rotates about the
same pivot and axis as in the URDF, with the URDF limits. `verify/` checks this
against RViz.

```
fusion/
  urdf_to_fusion_data.py        URDF (xacro) -> pibob_fusion.json          (runs on the Mac)
  pibob_fusion.json             generated; committed so Fusion works without Python
  PiBobAssembly/                the Fusion 360 script (PiBobAssembly.py + .manifest)
  preview_pose.py               poses/renders PiBob from the JSON alone (same motion as Fusion)
  verify/                       RViz comparison, mock-Fusion dry run, comparison PNGs, results.json
```

## Run it

1. **Regenerate the JSON** (needed only after the URDF changes; the committed copy matches `main`):
   ```bash
   uv run --with xacro --with numpy fusion/urdf_to_fusion_data.py
   ```
2. **Install the script in Fusion:** open **Utilities → Add-Ins → Scripts and Add-Ins**
   (or press Shift+S). Next to *My Scripts*, click **+**, choose
   `PiBob/fusion/PiBobAssembly`, then select **PiBobAssembly** and click **Run**.
   Fusion runs the script from where it sits in the repo. The script looks for
   `../pibob_fusion.json`, and the STLs are found through relative paths from the
   JSON (`../ros2/pibob_description/meshes/*.stl`). Keep `fusion/` inside the repo
   checkout. If you move it, the script asks you to pick the JSON, and it also
   accepts STLs copied into `fusion/meshes/`.
3. **Wait a few seconds.** A new untitled design opens. When the script finishes,
   a message box shows the joint self-check (see below). The same report is saved
   as `fusion/fusion_check.txt`.
4. **Save it:** use **File → Save** to store it in your Fusion hub, or
   **File → Export → .f3d** to get a local file.

## What you get

- **One component per URDF link** (15 of them: `base_link`, `shoulder_beam`, head ×3,
  and 5 per arm). Every link component sits at the link's zero-pose frame, so its
  **origin is the URDF joint pivot**.
- Inside each link component there is **one sub-component per URDF `<visual>`**,
  named `<link>__<Part>`, for example `r_upper_arm_link__Arm-Bicep`. It is placed at
  the visual `<origin>` and holds the STL as a mesh body, imported in mm inside a
  Base Feature. `upper_arm_link` has two sub-components, the bicep and the fixed
  elbow. The sub-components are *Ground to Parent*, so they move with their link.
- **As-built joints**, so nothing jumps when they are created:
  - 10 revolute joints about the URDF axis through the URDF pivot. Each has min/max
    limits from the URDF and a rest value of 0. Use *Drive Joints* or
    *Motion Study* to move them.
  - 4 rigid joints for the URDF `fixed` joints.
  - `base_link` is grounded.
- **Joint geometry is construction geometry only.** The pivot is the child link
  component's **origin point**. The axis is the child link component's X/Y/Z
  **construction axis**, or, for a non-principal URDF axis (none today), a
  construction sketch line inside the link component. No joint touches a mesh or
  BRep face or edge.

### Joint self-check (runs automatically in Fusion)

After building, the script drives each revolute joint to 0.3 rad and reads where
the child occurrence moved. It checks that the pivot stayed within 0.1 mm and that
a test point 100 mm from the pivot landed where the URDF says. Then it sets the
joint back to 0. If Fusion's positive direction is opposite to the URDF's for a
joint, the script notices, flips that joint's limits so the range of motion is
still correct, and reports `Fusion + = URDF -`. Any line reading `CHECK` or
`UNVERIFIED` means that joint needs a look.

**Angle signs:** the URDF uses −Y / −X axes for `r_shoulder_rot`, `r_shoulder_lift`
and both `bicep` joints. Fusion builds those joints on the +axis, so with Fusion's
usual right-hand convention their **Fusion angle is the URDF/ROS angle negated**.
The motion and limits are the same, only the displayed number's sign differs. The
self-check report confirms this for each joint.

## Editing / swapping a part

The joints use only link-component construction geometry, so you can replace a
part's geometry without breaking any joint:

1. Expand the link component, for example `r_upper_arm_link`, then its part
   sub-component, for example `r_upper_arm_link__Arm-Bicep`.
2. **Activate the part sub-component** and insert the new body into it, from a
   STEP/f3d via *Insert → Insert Derive / Insert into current design*, or by modelling
   it there directly. The sub-component's frame **is the STL's native frame**, the
   coordinates the part was modelled in. A STEP export of the same TinkerCAD or
   Fusion part therefore lands in the right place without any moving.
3. Delete the old mesh body. The joints are unaffected. Drive a joint to confirm.
4. If the new part is modelled in a different frame, move the *body* inside the
   part sub-component. Don't move the link component: its origin is the pivot.

To change a **pivot or axis**, edit the URDF, regenerate the JSON and re-run the
script. Moving a link component's origin in Fusion moves the pivot.

**Mesh bodies are not editable solids.** They are fine for rendering, interference
checks and motion studies, but you can't fillet, shell or sketch on them.
*Mesh → Convert Mesh* (prismatic/faceted, and it may need the mesh repaired first)
only suits simple, low-poly parts. Most of PiBob's printed brackets are
TinkerCAD-simple, so they may convert. The practical route for real edits is
still to re-export the original parts as STEP from TinkerCAD and swap them in as
above.

## Verification (what was checked, and how to re-run it)

All of these come from the same `pibob_fusion.json` the Fusion script reads.

1. **Numeric TF match with RViz.** `verify/run_verification.py` drives the
   `pibob-rviz` container's `/joint_states` (its `rviz_reference.py` replaces the
   slider GUI) at 12 poses: zero, each revolute joint alone at +45° (±20° for
   `shoulder_lift`, its limit), and one combined 10-joint pose. At each pose it
   compares every link's TF from `robot_state_publisher` with
   `preview_pose.pose_world()`. `pose_world()` rotates each child subtree about
   the joint's world pivot and axis, which is exactly what a Fusion revolute joint
   does. **Result: max error 0.0000 mm / 0.0000° for every link in every pose.**
2. **Visual match.** For each pose it screenshots RViz and renders the preview from
   the same camera (RViz Orbit model, 45° FOV). It saves
   `verify/<pose>.png` as *preview | RViz | overlay* for two views. In the preview,
   the moving subtree is orange and the pivot and axis are red. In the overlay, red
   is preview only and cyan is RViz only. **Silhouette IoU is 0.992–0.996 in every
   pose and view.** The remainder is edge anti-aliasing: no part detaches and
   nothing is mirrored.
3. **Mock-Fusion dry run.** `verify/mock_fusion_run.py` runs `PiBobAssembly.py`
   against a small mock of the adsk API that includes an as-built-joint solver. It
   checks that the script runs end to end and that every mesh vertex lands where
   the preview puts it: **0.000000 mm** at zero and at the combined pose, driven
   through the mock joints. It also checks the self-check and limit-sign logic under
   **both** possible Fusion sign conventions. It proves our code and maths, not
   that real Fusion accepts each call.
4. **Pivot vs servo holder** (`preview_pose.py --pivot-check`). This is the distance
   from each joint pivot to the nearest surface of the parent-side and child-side
   meshes, plus how far the pivot is outside the part's bounding envelope (0 means
   it's within the part, for example in its servo pocket, which is empty space in
   the STL):

   | joint | parent part: surface / outside envelope | child part: surface / outside envelope |
   |---|---|---|
   | head_pan | shoulder_beam 13.8 / 9.5 mm | Head-BottomServoHolder 4.4 / 0 mm |
   | head_tilt | Head-BottomServoHolder 5.8 / 5.8 mm | Head-MiddleServoHolder 5.8 / 0 mm |
   | shoulder_rot (L & R) | Shoulder-Top-ServoHolder 8.4 / 0 mm | Bracket 17.6 / 0 mm |
   | shoulder_lift (L & R) | Bracket 25.5 / 0 mm | Shoulder-ServoHolder 0.9 / 0 mm |
   | arm_rot (L & R) | Shoulder-ServoHolder 21.4 / 0 mm | Bicep+Elbow 4.5 / 0 mm |
   | bicep (L & R) | Bicep+Elbow 1.5 / 0 mm | Forearm 1.5 / 0 mm |

   Every pivot is inside the envelope of the part that carries its servo. The
   20–25 mm "surface" figures for `shoulder_lift` and `arm_rot` are distances into
   hollow holder or bracket interiors, not pivots floating outside the part. The
   head pivots sit a few mm above the part below them, which is where the servo
   horn meets the next holder. These are the URDF's own pivots, and the Fusion
   model uses them unchanged. The URDF comments say its offsets are approximate, so
   if a pivot looks off on the real robot, fix it in the URDF and regenerate.

Re-run everything:

```bash
./ros2/docker/dev.sh     # or: docker run -d --name pibob-rviz -p 6080:6080 pibob-ros2-rviz:humble
uv run --with numpy --with trimesh --with pillow fusion/verify/run_verification.py
uv run --with numpy --with trimesh --with pillow fusion/verify/mock_fusion_run.py
uv run --with numpy --with trimesh --with pillow fusion/preview_pose.py --pivot-check
```

`run_verification.py` replaces the container's slider GUI with its own
publisher. Restart the container to get the sliders back.

## Uncertain Fusion API spots (look here first if it errors)

The script was written against the documented API but has **not** been run in real
Fusion. Each of the following is wrapped or reported where possible:

1. **`comp.meshBodies.add(path, MeshUnits.MillimeterMeshUnit, baseFeature)` inside
   `baseFeature.startEdit()/finishEdit()` on a non-root, non-active component.** This
   is the mesh import. If it fails, try activating the component first, or switch
   the design to Direct (`design.designType = DirectDesignType`). In a Direct design
   the base feature isn't needed.
2. **`asBuiltJoints.createInput(child, parent, None)` for rigid joints.** The docs
   say geometry is optional for rigid joints. The script falls back to the child's
   origin point if Fusion rejects `None`.
3. **`setAsRevoluteJointMotion(JointDirections.CustomJointDirection, <construction-axis proxy>)`.**
   The axis is the child link's construction axis via `createForAssemblyContext`. If
   Fusion rejects a construction-axis proxy here, a construction sketch line (the
   script's non-principal fallback path) is the alternative.
4. **Joint sign convention and `jointMotion.rotationValue` updating `occurrence.transform2`
   immediately** during the self-check. The script calls `adsk.doEvents()` after
   setting the value. If the transform doesn't update, every joint reports
   `CHECK ... motion err` with ~0 pivot error. The joints are still fine; only the
   check couldn't see the move. Verify one by hand with Drive Joints.
5. **`occurrence.isGroundToParent`** on the part sub-occurrences (wrapped in
   try/except, so it's cosmetic). Without it you could drag a part away from its
   link by hand.
6. **The manifest** uses `"autodeskProduct": "Fusion"`, which current Fusion writes.
   Very old builds expected `"Fusion360"`.
7. Units: all API lengths are **cm** (metres × 100, done in `matrix_from_json`).
   Matrices are filled with `setCell(row, col)`, so there is no row- or
   column-major ambiguity. Angles are radians.

## Known limits

- STLs import as mesh bodies. See *Editing / swapping a part*.
- There are no materials or appearances. URDF colours are in the JSON (`rgba`) but
  aren't applied.
- Collisions and inertias aren't used, since the URDF has none.
- The Fusion model is the URDF's zero pose, which is PiBob's home pose (every servo
  at 90°).

# Finger Description
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This C++ ROS2 package contains functionality visualizing the finger and loading it into `rviz2`.

#### Software Structure
* config/ - Contains visualization param and `rviz2` config files.
* launch/ - Contains all launch files.
* meshes/ - Finger meshes
* urdf/ - Finger URDF files for visualization.

#### Export from Onshape Instructions
1. Export full assembly from onshape as STL file
2. Import as STL to blender in `segment_linkage.blend`
3. Click on full STL -> `Edit mode -> A -> P -> Separate by loose parts`
4. Use wireframe and join meshes `(Ctrl + J)` to chunk mesh into 5 links
    * Name them: `base_link, mcp_link, proximal_phalanx, middle_phalanx, distal_phalanx`
    * Put these into a collection titled `visual`
5. Ensure `base_link` is at (0,0,0)
6. Select each mesh, look at incoming joint axis.
    * Go to edit mode
    * Select opposite faces on shaft
    * `Shift + S -> 3D Cursor to selected`
    * Back to object mode -> `Object -> Origin to 3D Cursor`
    * `Shift + A -> Empty -> Arrow`
    * Align arrow with axis of rotation (do not translate, only rotate!)
    * NOTE: For middle_phalanx, look at outgoing joint axis of proximal_phalanx
7. Add the Empty objects to a collection named `joint_axes`
    * Name them: `mcp_splay, mcp_flex, pip_flex, dip_flex`
8. Duplicate visual meshes and move duplicates to `collision` collection
    * Change names to `col_<visual_mesh_name>`
9. For each visual mesh:
    * `Edit mode -> A -> X -> Limited dissolve`
10. For each collision mesh:
    * `Edit mode -> A -> Mesh -> Convex Hull -> X -> Limited dissolve`
11. Go to middle link -> `Edit mode -> Select all -> Separate by loose parts`
12. Rename exterior members `left_bar` and `right_bar`
    * There should now be 7 links
13. Add Empty arrow objects at each exterior bar pin joint, similar to step 6.
    * Names should be `left_bar_pip`, `left_bar_dip`, `right_bar_pip`, and `right_bar_dip`
14. Set origin of each exterior bar to the pin join on the `proximal_phalanx`
15. Finally play with each of the part origins and joint origins. 
    * Try to center each joint Empty arrow and linkage origin.
    * This step is important for the linkage constraints. 
16. Run script from scripting tab in blender
    * Make sure to set up paths and robot geometry in script

![](image/finger_rviz.png)
The visual geometry in rviz

![](image/col_rviz.png)
The collision geometry in rviz
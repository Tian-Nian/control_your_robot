import trimesh

# 读取 STL
input_files = ["meshes1/cam_Link.STL", "meshes1/cam_support_Link.STL"]
# input_files = ["meshes1/gripper_base.STL", "meshes1/base_link.STL", "meshes1/Claw_Link_L.STL", "meshes1/Claw_Link_R.STL"] # 
# for i in range(1,7):
#     input_files.append(f"meshes1/Link{i}.STL" )

for f in input_files:
    mesh = trimesh.load(f)
    # 给 mesh 加上一个统一颜色 (黑色)
    # mesh.visual.vertex_colors = [0, 0, 0, 255]  # RGBA
    mesh.visual.vertex_colors = [209, 209, 209, 255]  # RGBA

    output_file = f.rsplit(".", 1)[0] + ".glb"
    # 导出为 GLB
    mesh.export(output_file)
    print(output_file)

import trimesh
import sys

def convert_glb_to_stl(input_file, output_file=None):
    # 加载 glb 文件
    mesh = trimesh.load(input_file, force='mesh')
    
    # 默认输出名
    if output_file is None:
        output_file = input_file.rsplit(".", 1)[0] + ".stl"
    
    # 导出 stl
    mesh.export(output_file)
    print(f"转换完成: {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python convert_glb_to_stl.py model.glb [output.stl]")
    else:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else None
        convert_glb_to_stl(input_file, output_file)


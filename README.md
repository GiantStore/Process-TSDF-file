# Process-TSDF-file
读取sdf文件得到TSDF Volume，并生成点云和Mesh

程序逻辑:
+ 从.sdf文件中加载出TSDF Volume，大小为(nx, ny, nz)的array
+ TSDF Volume生成点云
+ TSDF Volume生成TriangleMesh



**核心函数`skimage.measure.marching_cubes_lewiner`找3D Volume等值面**

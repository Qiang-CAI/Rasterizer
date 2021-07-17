
//created by CQ  2021.05.24

#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

#include "../eigen/Eigen/Eigen"
class triangle
{
public:
	triangle();
	void set_Vertex(int ind, Eigen::Vector3f ver);
	void set_Normal(int ind, Eigen::Vector3f n);
	void set_Color(int ind, float r, float g, float b);
	void set_TexCoord(int ind, float u, float v);

	Eigen::Vector3f get_Color()const { return color[0] * 255; };
	std::array<Eigen::Vector4f, 3> to_Vector4()const;
	bool operator < (const triangle& a)const;
public:
	Eigen::Vector3f vertex[3];			//三角形坐标顶点
	Eigen::Vector3f color[3];			//顶点颜色
	Eigen::Vector2f tex_coords[3];		//纹理
	Eigen::Vector3f normal[3];			//法线
	bool is_opaque;
	const float blender = 0.7f;
};
#endif

#include "Triangle.h"
#include <iostream>
#include <algorithm>
#include <array>

triangle::triangle()
{
	vertex[0] << 0, 0, 0;
	vertex[1] << 0, 0, 0;
	vertex[2] << 0, 0, 0;

	color[0] << 0, 0, 0;
	color[1] << 0, 0, 0;
	color[2] << 0, 0, 0;

	tex_coords[0] << 0, 0;
	tex_coords[1] << 0, 0;
	tex_coords[2] << 0, 0;

	normal[0] << 0, 0, 0;
	normal[0] << 0, 0, 0;
	normal[0] << 0, 0, 0;
}

void triangle::set_Vertex(int ind, Eigen::Vector3f ver)
{
	vertex[ind] = ver;
}
void triangle::set_Normal(int ind, Eigen::Vector3f n)
{
	normal[ind] = n;
}
void triangle::set_Color(int ind, float r, float g, float b)
{
	if ((r < 0.0) || (r > 255.0) || (g < 0.0) || (g > 255.0) || (b < 0.0) || (b > 255.0))
	{
		std::cout << "Error Invalid color input" << std::endl;
		exit(-1);
	}
	color[ind] = Eigen::Vector3f(static_cast<float>(r / 255), static_cast<float>(g / 255), static_cast<float>(b / 255));
	return;
}
void triangle::set_TexCoord(int ind, float u, float v)
{
	tex_coords[ind] = Eigen::Vector2f(u, v);
}

std::array<Eigen::Vector4f, 3> triangle::to_Vector4()const
{
	std::array<Eigen::Vector4f, 3> res;
	std::transform(std::begin(vertex), std::end(vertex),res.begin(), [](auto &vec)
	{
		return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.0f);
	});
	return res;
}

bool triangle::operator < (const triangle& a) const
{
	if (this->is_opaque)
	{
		return true;
	}
	else if (a.is_opaque)
	{
		return false;
	}
	return true;
}
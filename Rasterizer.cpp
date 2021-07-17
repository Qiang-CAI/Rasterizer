
#include "Rasterizer.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace CQ;

rasterizer::rasterizer(int w, int h, bool flag):width(w),height(h),MSAA_flag(flag)
{
	//初始化buffer_size:
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

//导入坐标、颜色、下标 --> 缓冲区：
pos_buff_id rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);
	return { id };
}
col_buff_id rasterizer::load_colors(const std::vector<Eigen::Vector3f> &colors)
{
	auto id = get_next_id();
	col_buf.emplace(id, colors);
	return { id };
}
ind_buff_id rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);
	return { id };
}
opa_buff_id rasterizer::load_opaque(const std::vector<bool> &opaque)
{
	auto id = get_next_id();
	opa_buf.emplace(id, opaque);
	return { id };
}

//导入MVP矩阵：
void rasterizer::set_model(const Eigen::Matrix4f &m)
{
	model = m;
}
void rasterizer::set_view(const Eigen::Matrix4f &v)
{
	view = v;
}
void rasterizer::set_projection(const Eigen::Matrix4f &p)
{
	projection = p;
}

void rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
	auto ind = get_index((int)point.x(),(int)point.y());
	frame_buf[ind] = color;
}

int rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

Eigen::Vector3f rasterizer::get_framebuffer_color(int x, int y)
{
	auto ind = get_index(x, y);
	return frame_buf[ind];
}

bool rasterizer::inside_triangle(float x, float y, const Eigen::Vector3f *_v)
{
	Eigen::Vector3f point;
	point << x, y, _v[0].z();

	auto dir_0 = (point - _v[0]).cross(_v[1] - _v[0]);
	auto dir_1 = (point - _v[1]).cross(_v[2] - _v[1]);
	auto dir_2 = (point - _v[2]).cross(_v[0] - _v[2]);

	bool flag = true;
	if (dir_0.dot(dir_1) < 0 || dir_1.dot(dir_2) < 0 || dir_2.dot(dir_0) < 0)
	{
		flag = false;
	}
	return flag;
}

//重心坐标计算：
static std::tuple<float, float, float> computeBarycentric(float x, float y,const Eigen::Vector3f *v)
{
	float alpha = (-(x - v[1].x()) * (v[2].y() - v[1].y()) + (y - v[1].y()) * (v[2].x() - v[1].x()))
		/ (-(v[0].x() - v[1].x()) * (v[2].y() - v[1].y()) + (v[0].y() - v[1].y()) * (v[2].x() - v[1].x()));
	float beta = (-(x - v[2].x()) * (v[0].y() - v[2].y()) + (y - v[2].y()) * (v[0].x() - v[2].x()))
		/ (-(v[1].x() - v[2].x())*(v[0].y() - v[2].y()) + (v[1].y() - v[2].y())*(v[0].x() - v[2].x()));
	float gamma = 1.0 - alpha - beta;
	return { alpha,beta,gamma };
}

void rasterizer::opaque_triangle_pass(const triangle &t)
{
	auto v = t.to_Vector4();

	//bounding box:
	float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
	float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
	float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
	float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

	min_x = static_cast<int>(std::floor(min_x));
	max_x = static_cast<int>(std::ceil(max_x));
	min_y = static_cast<int>(std::floor(min_y));
	max_y = static_cast<int>(std::ceil(max_y));

	if (MSAA_flag)
	{
		std::vector<Eigen::Vector2f> ms_pos
		{
			{0.25,0.25},
			{0.75,0.25},
			{0.25,0.75},
			{0.75,0.75},
		};
		for (int x = min_x; x <= max_x; ++x)
		{
			for (int y = min_y; y <= max_y; ++y)
			{
				float minDepth = FLT_MAX;

				//统计在三角形内部的点：
				int count = 0;
				for (int i = 0; i < 4; ++i)
				{
					float _x = (float)(x)+ms_pos[i][0];
					float _y = (float)(y)+ms_pos[i][1];
					if (inside_triangle(_x, _y, t.vertex))
					{
						std::tuple<float, float, float> Barycent = computeBarycentric(_x, _y, t.vertex);
						float alpha = std::get<0>(Barycent);
						float beta = std::get<1>(Barycent);
						float gamma = std::get<2>(Barycent);
						//正则化项：
						float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
						//重心插值求z
						float z = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
						z *= w_reciprocal;
						minDepth = std::min(z, minDepth);
						++count;
					}
				}
				if(count != 0)
				{
					int index = get_index(x, y);
					if (minDepth <= depth_buf[index])
					{
						//深度写入：
						depth_buf[index] = minDepth;
						Eigen::Vector3f point;
						point << x, y, 0;

						Eigen::Vector3f color = t.get_Color() * count / 4.0;

						set_pixel(point, color);
					}
				}

			}
		}
	}
	else
	{
		for (int x = min_x; x <= max_x; ++x)
		{
			for (int y = min_y; y <= max_y; ++y)
			{
				if (inside_triangle(x + 0.5f, y + 0.5f, t.vertex))
				{
					int index = get_index(x, y);
					std::tuple<float, float, float> Barycent = computeBarycentric(x + 0.5f, y + 0.5f, t.vertex);
					float alpha = std::get<0>(Barycent);
					float beta = std::get<1>(Barycent);
					float gamma = std::get<2>(Barycent);
					//正则化项：
					float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					//重心插值求z
					float z = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z *= w_reciprocal;
					if (z <= depth_buf[index])
					{
						depth_buf[index] = z;
						Eigen::Vector3f point;
						point << x, y, 0;
						set_pixel(point, t.get_Color());
					}
				}
			}
		}
	}
}

void rasterizer::blender_triangle_pass(const triangle &t)
{
	auto v = t.to_Vector4();
	float blender = t.blender;
	//bounding box:
	float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
	float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
	float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
	float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

	min_x = static_cast<int>(std::floor(min_x));
	max_x = static_cast<int>(std::ceil(max_x));
	min_y = static_cast<int>(std::floor(min_y));
	max_y = static_cast<int>(std::ceil(max_y));

	if (MSAA_flag)
	{
		std::vector<Eigen::Vector2f> ms_pos
		{
			{0.25,0.25},
			{0.75,0.25},
			{0.25,0.75},
			{0.75,0.75},
		};
		for (int x = min_x; x <= max_x; ++x)
		{
			for (int y = min_y; y <= max_y; ++y)
			{
				float minDepth = FLT_MAX;

				//统计在三角形内部的点：
				int count = 0;
				for (int i = 0; i < 4; ++i)
				{
					float _x = (float)(x)+ms_pos[i][0];
					float _y = (float)(y)+ms_pos[i][1];
					if (inside_triangle(_x, _y, t.vertex))
					{
						std::tuple<float, float, float> Barycent = computeBarycentric(_x, _y, t.vertex);
						float alpha = std::get<0>(Barycent);
						float beta = std::get<1>(Barycent);
						float gamma = std::get<2>(Barycent);
						//正则化项：
						float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
						//重心插值求z
						float z = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
						z *= w_reciprocal;
						minDepth = std::min(z, minDepth);
						++count;
					}
				}
				if (count != 0)
				{
					int index = get_index(x, y);
					if (minDepth <= depth_buf[index])
					{
						Eigen::Vector3f point;
						point << x, y, 0;
						Eigen::Vector3f color = (t.get_Color()*blender + (1 - blender)*get_framebuffer_color(x, y)*count / 4.0);
						set_pixel(point, color);
					}
				}
			}
		}
	}
}

static Eigen::Vector4f to_vec4(const Eigen::Vector3f &v3, float w)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rasterizer::draw(pos_buff_id pos_buffer, ind_buff_id ind_buffer, col_buff_id col_buffer, opa_buff_id opa_buffer, Primitive type)
{
	switch (type)
	{
	case Primitive::Line:
		break;
	case Primitive::Triangel:
		draw_triangle(pos_buffer, ind_buffer, col_buffer, opa_buffer);
		break;
	default:
		break;
	}
}

void rasterizer::draw_triangle(pos_buff_id pos_buffer, ind_buff_id ind_buffer, col_buff_id col_buffer, opa_buff_id opa_buffer)
{
	auto &pos = pos_buf[pos_buffer.pos_id];
	auto &ind = ind_buf[ind_buffer.ind_id];
	auto &col = col_buf[col_buffer.col_id];
	auto &opa = opa_buf[opa_buffer.opa_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f MVP = model * view * projection;
	std::deque<triangle> triangles;
	int opaque_id = 0;
	for (auto &in : ind)
	{
		triangle t;
		Eigen::Vector4f v[] =
		{
			MVP * to_vec4(pos[in[0]],1.0f),
			MVP * to_vec4(pos[in[1]],1.0f),
			MVP * to_vec4(pos[in[2]],1.0f),
		};
		//归一化：
		for (auto &vec : v)
		{
			vec /= vec.w();
		}
		//视口变换：
		for (auto &vert : v)
		{
			vert.x() = 0.5*width*(vert.x() + 1);
			vert.y() = 0.5*height*(vert.y() + 1);
			vert.z() = vert.z() * f1 + f2;
		}
		for (int i = 0; i < 3; ++i)
		{
			t.set_Vertex(i, v[i].head<3>());
		}
		auto col_a = col[in[0]];
		auto col_b = col[in[1]];
		auto col_c = col[in[2]];
		t.set_Color(0, col_a[0], col_b[1], col_c[2]);
		t.set_Color(1, col_a[0], col_b[1], col_c[2]);
		t.set_Color(2, col_a[0], col_b[1], col_c[2]);
		t.is_opaque = opa[opaque_id++];

		if (t.is_opaque)
		{
			triangles.push_front(t);
		}
		else
		{
			triangles.push_back(t);
		}
	}
	for (auto &t : triangles)
	{
		if (t.is_opaque)
		{
			opaque_triangle_pass(t);
		}
		else
		{
			blender_triangle_pass(t);
		}
	}
}
void rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
	//3D models or hollow triangle TO DO:

}

void rasterizer::clear(Buffers buff)
{
	if ((buff & Buffers::Color) == Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0,0,0 });
	}
	if ((buff & Buffers::Deepth) == Buffers::Deepth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}
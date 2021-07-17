
//created by CQ  2021.05.25

#ifndef _RASTERIZER_HPP_
#define _RASTERIZER_HPP_

#include "Triangle.h"

namespace CQ
{
	enum class Buffers
	{
		Color = 1,
		Deepth = 2
	};
	inline Buffers operator |(Buffers a, Buffers b)
	{
		return Buffers(static_cast<int>(a) | static_cast<int>(b));
	}
	inline Buffers operator&(Buffers a, Buffers b)
	{
		return Buffers(static_cast<int>(a) & static_cast<int>(b));
	}
	enum class Primitive
	{
		Line,
		Triangel
	};

	struct pos_buff_id
	{
		int pos_id = 0;
	};
	struct col_buff_id
	{
		int col_id = 0;
	};
	struct ind_buff_id
	{
		int ind_id = 0;
	};
	struct opa_buff_id
	{
		int opa_id = 0;
	};

	class rasterizer
	{
	public:
		rasterizer(int w, int h, bool flag);
		pos_buff_id load_positions(const std::vector<Eigen::Vector3f> &positions);
		col_buff_id load_colors(const std::vector<Eigen::Vector3f> &colors);
		ind_buff_id load_indices(const std::vector<Eigen::Vector3i> &indices);
		opa_buff_id load_opaque(const std::vector<bool> &opaque);

		void set_model(const Eigen::Matrix4f &m);
		void set_view(const Eigen::Matrix4f &v);
		void set_projection(const Eigen::Matrix4f &p);
		void set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color);

		int get_index(int x, int y);
		int get_next_id() { return next_id++; };
		std::vector<Eigen::Vector3f> &get_frame_buffer() { return frame_buf; };
		Eigen::Vector3f get_framebuffer_color(int x, int y);

		void opaque_triangle_pass(const triangle &t);			//光栅化
		void blender_triangle_pass(const triangle &t);
		bool inside_triangle(float x, float y, const Eigen::Vector3f *_v);	//像素点是否在三角形内部
		//开放绘制接口：
		void draw(pos_buff_id pos_buffer, ind_buff_id ind_buffer, col_buff_id col_buffer, opa_buff_id opa_buffer, Primitive type);
		void clear(Buffers buff);

	private:
		void draw_triangle(pos_buff_id pos_buffer, ind_buff_id ind_buffer, col_buff_id col_buffer, opa_buff_id opa_buffer);
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
	private:
		int next_id = 0;
		int width;
		int height;
		bool MSAA_flag;

		Eigen::Matrix4f model;
		Eigen::Matrix4f view;
		Eigen::Matrix4f projection;

		std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
		std::map<int, std::vector<Eigen::Vector3f>> col_buf;
		std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
		std::map<int, std::vector<bool>> opa_buf;

		std::vector<Eigen::Vector3f> frame_buf;
		std::vector<float> depth_buf;
	};
}
#endif
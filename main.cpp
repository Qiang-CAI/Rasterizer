
#include "Global.h"
#include "Rasterizer.hpp"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace CQ;

Eigen::Matrix4f set_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	return model;
}
Eigen::Matrix4f set_view_matrix(Eigen::Vector3f camera_position)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f trans;
	trans << 1, 0, 0, -camera_position[0],
		0, 1, 0, -camera_position[1],
		0, 0, 1, -camera_position[2],
		0, 0, 0, 1;
	view = trans * view;
	return view;
}
Eigen::Matrix4f set_project_matrix(float eye_fov, float aspect, float zNear, float zFar)
{
	Eigen::Matrix4f project = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f trans;
	trans << 1.0f / aspect * tan(eye_fov / 180.0f*MY_PI), 0, 0, 0,
		0, 1.0f / tan(eye_fov / 180.0f*MY_PI), 0, 0,
		0, 0, (-zFar - zNear) / (zFar - zNear), (2.0f*zFar*zNear) / (zNear - zFar),
		0, 0, -1, 0;
	project = trans * project;
	return project;
}

int main()
{
	float angle = 0;
	std::string filename = "msaa_output.png";
	rasterizer r(700, 700, true);
	Eigen::Vector3f eye_pos = { 0,0,20 };
	std::vector<Eigen::Vector3f> positions
	{
		{2,0,-6},
		{0,2,-6},
		{-2,0,-6},
		{3.5,-1,-10},
		{2.5,2.5,-10},
		{-1,0.5,-4},
	};
	std::vector<Eigen::Vector3i> inds
	{
		{0,1,2},
		{3,4,5},
	};
	std::vector<Eigen::Vector3f> colors
	{
		{255.0,255.0,255.0},
		{255.0,255.0,255.0},
		{255.0,255.0,255.0},
		{255.0,48.0,48.0},
		{255.0,48.0,48.0},
		{255.0,48.0,48.0},
	};
	std::vector<bool> opaque
	{
		false,
		true
	};
	auto pos_id = r.load_positions(positions);
	auto ind_id = r.load_indices(inds);
	auto col_id = r.load_colors(colors);
	auto opa_id = r.load_opaque(opaque);

	int key = 0;
	int frame_count = 0;
	while (key != 27) //Esc
	{
		r.clear(Buffers::Color | Buffers::Deepth);
		r.set_model(set_model_matrix(angle));
		r.set_view(set_view_matrix(eye_pos));
		r.set_projection(set_project_matrix(45, 1, 0.1, 50));
		r.draw(pos_id, ind_id, col_id, opa_id, Primitive::Triangel);

		cv::Mat image(700, 700, CV_32FC3, r.get_frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imshow("image", image);
		cv::imwrite(filename, image);
		key = cv::waitKey(10);

		std::cout << "frame count" << frame_count++ << std::endl;
	}
	return 0;
}

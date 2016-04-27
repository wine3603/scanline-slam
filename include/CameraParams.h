/*******************************************************************
 * Copyright (c) 2013 Kiyoshi Irie
 *
 * @file CameraParams.h
 * @brief Camera Params
 * @author Kiyoshi Irie
 * @date 2013-09-17
 *******************************************************************/
#pragma once

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <opencv2/opencv.hpp>

struct CameraExtrinsicParams
{
	double r1, r2, r3;
	double tx, ty, tz;

	CameraExtrinsicParams()
		: r1(0), r2(0), r3(0), tx(0), ty(0), tz(0)
	{
	}
	CameraExtrinsicParams(double r1_, double r2_, double r3_, double tx_, double ty_, double tz_)
		: r1(r1_), r2(r2_), r3(r3_), tx(tx_), ty(ty_), tz(tz_)
	{
	}
	cv::Mat getRvec() const {
		cv::Mat rvec = cv::Mat(3, 1, CV_64FC1);
		rvec.at<double>(0, 0) = r1;
		rvec.at<double>(1, 0) = r2;
		rvec.at<double>(2, 0) = r3;
		return rvec.clone();
	}
	cv::Mat getTvec() const {
		cv::Mat tvec = cv::Mat(3, 1, CV_64FC1);
		tvec.at<double>(0, 0) = tx;
		tvec.at<double>(1, 0) = ty;
		tvec.at<double>(2, 0) = tz;
		return tvec.clone();
	}
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		(void)version;
		ar & BOOST_SERIALIZATION_NVP(r1);
		ar & BOOST_SERIALIZATION_NVP(r2);
		ar & BOOST_SERIALIZATION_NVP(r3);
		ar & BOOST_SERIALIZATION_NVP(tx);
		ar & BOOST_SERIALIZATION_NVP(ty);
		ar & BOOST_SERIALIZATION_NVP(tz);
	}
	void saveToXML(const std::string &filename){

		std::ofstream ofs(filename.c_str());
		boost::archive::xml_oarchive oar(ofs);

		oar << boost::serialization::make_nvp("cameraextrinsic", *this);
	}

	static CameraExtrinsicParams loadFromXML(const std::string &filename){
		CameraExtrinsicParams param;
		std::ifstream ifs(filename.c_str());
		try {
			boost::archive::xml_iarchive iar(ifs);
			iar >> boost::serialization::make_nvp("cameraextrinsic", param);
		} catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

		return param;
	}
};

struct CameraIntrinsicParams
{
	double focal_x, focal_y;
	double cx, cy;
	double k1, k2, p1, p2;

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		(void)version;
		ar & BOOST_SERIALIZATION_NVP(focal_x);
		ar & BOOST_SERIALIZATION_NVP(focal_y);
		ar & BOOST_SERIALIZATION_NVP(cx);
		ar & BOOST_SERIALIZATION_NVP(cy);
		ar & BOOST_SERIALIZATION_NVP(k1);
		ar & BOOST_SERIALIZATION_NVP(k2);
		ar & BOOST_SERIALIZATION_NVP(p1);
		ar & BOOST_SERIALIZATION_NVP(p2);
	}
	void saveToXML(const std::string &filename){

		std::ofstream ofs(filename.c_str());
		boost::archive::xml_oarchive oar(ofs);

		oar << boost::serialization::make_nvp("cameraintrinsic", *this);
	}

	static CameraIntrinsicParams loadFromXML(const std::string &filename){
		CameraIntrinsicParams param;

		std::ifstream ifs(filename.c_str());
		try {
			boost::archive::xml_iarchive iar(ifs);
			iar >> boost::serialization::make_nvp("cameraintrinsic", param);
		} catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

		return param;
	}
	CameraIntrinsicParams() {
	}
	cv::Mat getCameraMatrix() const {
		std::vector<double> cammatvec;
		cammatvec.push_back(focal_x);
		cammatvec.push_back(0);
		cammatvec.push_back(cx);
		cammatvec.push_back(0);
		cammatvec.push_back(focal_y);
		cammatvec.push_back(cy);
		cammatvec.push_back(0);
		cammatvec.push_back(0);
		cammatvec.push_back(1);
		cv::Mat cammat = cv::Mat(3, 3, CV_64FC1, &cammatvec[0]);
		return cammat.clone();
	}
	cv::Mat getDistCoeffs() const {
		std::vector<double> distcoeffvec;
		distcoeffvec.push_back(k1);
		distcoeffvec.push_back(k2);
		distcoeffvec.push_back(p1);
		distcoeffvec.push_back(p2);
		cv::Mat distmat = cv::Mat(1, 4, CV_64FC1, &distcoeffvec[0]);
		return distmat.clone();
	}
};


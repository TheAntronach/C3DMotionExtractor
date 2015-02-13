#include "stdafx.h"
#include "uuc3d.hpp"
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <fstream>
#include <map>
#include <atlimage.h>
#include <numeric>

#define _USE_MATH_DEFINES
#include <math.h>

using std::cerr;
using std::vector;
using UuIcsC3d::SpacePaddedString;
using UuIcsC3d::DataPoint3d;
using namespace boost::filesystem;

vector<float> getTargetLetter(std::string letter)
{
	vector<float> target(2);
	if (letter == "a") {
		target[0] = 2;
		target[1] = 0;
	} else if (letter == "b") {
		target[0] = 2;
		target[1] = -2;
	} else if (letter == "c") {
		target[0] = 0;
		target[1] = -2;
	} else if (letter == "d") {
		target[0] = -2;
		target[1] = -2;
	} else if (letter == "e") {
		target[0] = -2;
		target[1] = 0;
	} else if (letter == "f") {
		target[0] = -2;
		target[1] = 2;
	} else if (letter == "g") {
		target[0] = 0;
		target[1] = 2;
	} else if (letter == "h") {
		target[0] = 2;
		target[1] = 2;
	}
	return target;
}

vector<std::pair<float, float>> getPoints (boost::property_tree::ptree pt, int from, int to)
{
	vector<std::pair<float, float>> points(to - from + 2); // one before and one after
	std::pair<float, float> point;
	std::string walker;
	std::string::size_type sz;
	float fPointX, fPointY;

	walker = pt.get_child("trial.id").data();
	walker = walker.substr(0,2);

	int n = 1;
	int frame = 1;
	int vectorCounter = 0;
	bool start = false;
	for(auto motions : pt.get_child("subjects." + walker + ".motion")) 
	{
		for (auto motion : motions.second) 
		{
			if (frame == (from-1))
				start = true;
			if (frame == (to + 1)) 
				start = false;
			if (n == 1)
			{
				fPointX = std::stof(motion.second.get_value<std::string>(), &sz);
				n = 2;
			}
			else if (n == 2)
			{
				fPointY = std::stof(motion.second.get_value<std::string>(), &sz);
				n = 3;
			}
			else if (n == 3)
			{
				if (start)
				{
					point.first = fPointX;
					point.second = fPointY;
					points[vectorCounter] = point;
					vectorCounter++;
				}
				n = 1;
				frame++;
			}
		}
	}
	return points;
}


vector<float> getAngles (boost::property_tree::ptree pt, int from, int to)
{
	vector<float> angles(to - from + 2); // one before and one after
	std::string walker;
	std::string::size_type sz;
	float fAngle;
	float radianAngleA;

	walker = pt.get_child("trial.id").data();
	walker = walker.substr(0,2);

	int n = 1;
	int frame = 1;
	int vectorCounter = 0;
	bool start = false;
	for(auto motions : pt.get_child("subjects." + walker + ".motion")) 
	{
		for (auto motion : motions.second) 
		{
			if (frame == (from-1))
				start = true;
			if (frame == (to + 1)) 
				start = false;
			if (n == 1)
				n = 2;
			else if (n == 2)
				n = 3;
			else if (n == 3)
			{
				if (start)
				{
					fAngle = std::stof(motion.second.get_value<std::string>(), &sz);
					radianAngleA = (fAngle*(M_PI/180)); // to radians
					angles[vectorCounter] = radianAngleA;
					vectorCounter++;
				}
				n = 1;
				frame++;
			}
		}
	}
	return angles;
}

void getVelocity(boost::property_tree::ptree pt, int from, int to)
{
	vector<std::pair<float, float>> points;
	points = getPoints(pt, from, to);
	std::ofstream Velocities;
    Velocities.open (pt.get_child("trial.id").data() + "Velocities.txt");
	int length = to-from;
	float velocity, lengthX, lengthY, distance;
	for (int n = 1; n <= length;n++)
	{
		lengthX = points[n+1].first - points[n-1].first;
		lengthY = points[n+1].second - points[n-1].second;

		distance = std::sqrtf(lengthX * lengthX + lengthY * lengthY);
		velocity = distance/2; // in m/frame
		velocity = velocity * 100; // in m/s 

		Velocities << velocity << std::endl;
	}

	Velocities.close();
}

void getAngleSpeed(boost::property_tree::ptree pt, int from, int to)
{
	vector<float> angles;
	angles = getAngles(pt, from, to);
	std::ofstream AngleSpeed;
    AngleSpeed.open (pt.get_child("trial.id").data() + "AngleSpeed.txt");
	int length = to-from;
	float angleSpeed, angleSpeedDeg;
	for (int n = 1; n <= length;n++)
	{
		angleSpeed = (angles[n+1] - angles[n-1]) / 2; // radians/frame
		angleSpeedDeg = angleSpeed*(180/M_PI); // in degrees/frame
		angleSpeedDeg = angleSpeedDeg * 100; // to degrees/sec;
		AngleSpeed << angleSpeedDeg << std::endl;
	}

	AngleSpeed.close();
}

void getLookAtTarget(boost::property_tree::ptree pt, int from, int to)
{
	std::ofstream lookAtTarget;
    lookAtTarget.open (pt.get_child("trial.id").data() + "LookAtTarget.txt");

	bool start = false;
	std::string motionX, motionY, walker, targetLetter, outputAngle;
	std::string outputAngleBeta, outputAngleTheta, outputAngleRho;
	std::string::size_type sz;
	float fMotionX, fMotionY, fAngle;
	float radianAngleA, walkerVecX, walkerVecY, targetVecX, targetVecY, lengthTargetVec, lengthWalkerVec, normTargetVecX, normTargetVecY, dotProduct, angle, angleDeg;
	vector<float> target(2);
	int yes = 0, no = 0, total = 0;
	walker = pt.get_child("trial.id").data();
	walker = walker.substr(0,2);
	targetLetter = pt.get_child("trial.actual_goal").data();
	target = getTargetLetter(targetLetter);
	int n = 1;
	int frame = 1;

	std::cout<< "walker: " << walker << " target: " << targetLetter << std::endl;
	
	// only do the walker:
	for(auto motions : pt.get_child("subjects." + walker + ".motion")) 
	{
		for (auto motion : motions.second) 
		{
			if (frame == from)
				start = true;
			if (frame == to)
				start = false;

			if (n == 1)
			{
				motionX = motion.second.get_value<std::string>();
				n = 2;
			} 
			else if (n == 2) 
			{
				motionY = motion.second.get_value<std::string>();
				n = 3;
			}
			else if (n == 3)
			{
				if (start)
				{
					fMotionX = std::stof(motionX, &sz);
					fMotionY = std::stof(motionY, &sz);
					fAngle = std::stof(motion.second.get_value<std::string>(), &sz);
					radianAngleA = (fAngle*(M_PI/180));

					walkerVecX = -sinf(radianAngleA);
					walkerVecY = cosf(radianAngleA);
				
					targetVecX = target[0] - fMotionX;
					targetVecY = target[1] - fMotionY;

					lengthTargetVec = std::sqrtf(targetVecX * targetVecX + targetVecY * targetVecY);
					lengthWalkerVec = std::sqrtf(walkerVecX * walkerVecX + walkerVecY * walkerVecY);

					normTargetVecX = targetVecX / lengthTargetVec;
					normTargetVecY = targetVecY / lengthTargetVec;

					dotProduct = walkerVecX * normTargetVecX + walkerVecY * normTargetVecY;

					angle = std::acosf(dotProduct / 1); // length is 1, they are unit vectors

					angleDeg = angle*(180/M_PI);

					// lookAtTarget yes/no
					lookAtTarget << " dotProduct: " << dotProduct << " fAngle: " << fAngle << " angle: " << angle << " AngleDeg: " << angleDeg << std::endl;
				}
				n = 1;
				frame++;
				
			}
		}
		//std::cout<<"data: " << motions.second.data() << std::endl;
	}
	lookAtTarget.close();
}

boost::property_tree::ptree loadTree(std::string filename)
{
	boost::property_tree::ptree pt;
	std::ifstream file(filename, std::ios_base::in | std::ios_base::binary);
    try 
	{
        boost::iostreams::filtering_istream in;
        in.push(boost::iostreams::gzip_decompressor());
        in.push(file);
		boost::property_tree::read_json(in, pt);
    }
    catch(const boost::iostreams::gzip_error& e) {
         std::cout << e.what() << '\n';
    }

	return pt;

}

void open_folder(std:: string folder)
{
	path p(folder);
	if (!exists(p))
	{
		std::cout<< p << "does not exist";
		return;
	}
	if (!is_directory(p))
	{
		std::cout<< p << "is not a folder";
		return;
	}

	directory_iterator end_iter;
	for (directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
	{
		if (dir_itr->path().filename().extension().string() != ".gz")
		{
			std::cout<<"File " << dir_itr->path().filename().string() << " is not a .gz file";
			continue;
		}

		boost::property_tree::ptree pt;
		int from, to;
		try 
		{
			pt = loadTree(dir_itr->path().string());
			from = std::stoi(pt.get_child("trial.dense_movement_start").data());// start dense movement
			to = std::stoi(pt.get_child("trial.dense_movement_end").data()); // end dense movement

			getLookAtTarget(pt, from, to);
			getAngleSpeed(pt, from, to);
			getVelocity(pt, from, to);
		}
		catch (UuIcsC3d::OpenError const &err) 
		{
			cerr<<"An error occurred while processing file "<< err.filename();
			return;
		}
	}
}

// The main function checks the number of arguments and deals with exceptions
int main(int argc, char* argv[])
{
    if (argc<2) 
	{
		cerr<<"Expected folder as argument.\n";
		return 1;
    }
    try 
	{
		open_folder(argv[1]);
    } 
	catch (UuIcsC3d::OpenError const &err) 
	{
		cerr<<"An error occurred while opening folder "<<err.filename();
		return 2;
    }
    return 0;
}

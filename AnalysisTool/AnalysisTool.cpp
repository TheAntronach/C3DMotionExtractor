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

// The unchanged values for the target letters
vector<float> getTargetLetterOld(std::string letter)
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

// more accurate values for the position of the letters.
vector<float> getTargetLetter(std::string letter)
{
	vector<float> target(2);
	if (letter == "a") {
		target[0] = 3;
		target[1] = 0;
	} else if (letter == "b") {
		target[0] = 3;
		target[1] = -3;
	} else if (letter == "c") {
		target[0] = 0;
		target[1] = -3;
	} else if (letter == "d") {
		target[0] = -3;
		target[1] = -3;
	} else if (letter == "e") {
		target[0] = -3;
		target[1] = 0;
	} else if (letter == "f") {
		target[0] = -3;
		target[1] = 3;
	} else if (letter == "g") {
		target[0] = 0;
		target[1] = 3;
	} else if (letter == "h") {
		target[0] = 3;
		target[1] = 3;
	}
	return target;
}

vector<float> getCrossProduct(float Ax, float Ay, float Az, float Bx, float By, float Bz)
{
    vector<float> vec(3);
    vec[0] = (Ay*Bz)-(Az*By);
	vec[1] = (Az*Bx)-(Ax*Bz);
    vec[2] = (Ax*By)-(Ay*Bx);
    return vec;
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

void getBothVelocities(boost::property_tree::ptree pt, int from, int to)
{
	std::string tag = pt.get_child("trial.id").data();
	// create a csv file for all Trials together and one for the current trial
	std::ofstream AngularLinearVelocityAll , AngularLinearVelocityOne;
	AngularLinearVelocityAll.open("AllLinearAngularVelocities.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	AngularLinearVelocityOne.open(tag + "LinearAngularVelocities.csv");
	
	vector<std::pair<float, float>> points;
	points = getPoints(pt, from, to);

	vector<float> angles;
	angles = getAngles(pt, from, to);

	int length = to-from;
	float velocity, lengthX, lengthY, distance;
	float angleSpeed, angleSpeedDeg;
	for (int n = 1; n <= length;n++)
	{
		lengthX = points[n+1].first - points[n-1].first;
		lengthY = points[n+1].second - points[n-1].second;

		distance = std::sqrtf(lengthX * lengthX + lengthY * lengthY);
		velocity = distance/2; // in m/frame
		velocity = velocity * 100; // in m/s 

		angleSpeed = (angles[n+1] - angles[n-1]) / 2; // radians/frame
		angleSpeedDeg = angleSpeed*(180/M_PI); // in degrees/frame
		angleSpeedDeg = angleSpeedDeg * 100; // to degrees/sec;
		if (angleSpeedDeg <= 180.0 && angleSpeedDeg >= -180.0)
		{
			AngularLinearVelocityOne << velocity << ";" << std::abs(angleSpeedDeg) << std::endl;
			AngularLinearVelocityAll << velocity << ";" << std::abs(angleSpeedDeg) << std::endl;
		}
	}
	AngularLinearVelocityAll.close();
	AngularLinearVelocityOne.close();
}

float getLookAtTarget(boost::property_tree::ptree pt, int from, int to)
{
	// create the csv file of LookAtTarget
	std::ofstream lookAtTargetOne;
	std::ofstream lookAtTargetperWalker;
	std::ofstream lookAtTargetAll;
	std::ofstream lookAtTargetLetterA, lookAtTargetLetterB, lookAtTargetLetterC, lookAtTargetLetterD, lookAtTargetLetterE, lookAtTargetLetterF, lookAtTargetLetterG, lookAtTargetLetterH;

	bool start = false;
	std::string motionX, motionY, walker, targetLetter, outputAngle;
	std::string outputAngleBeta, outputAngleTheta, outputAngleRho;
	std::string::size_type sz;
	float fMotionX, fMotionY, fAngle;
	float radianAngleA, walkerVecX, walkerVecY, targetVecX, targetVecY, lengthTargetVec, lengthWalkerVec, normTargetVecX, normTargetVecY, dotProduct, angle, angleDeg;
	vector<float> target(2);
	vector<float> crossproduct(3);
	int yes = 0, no = 0, total = 0;
	walker = pt.get_child("trial.id").data();
	walker = walker.substr(0,2);
	targetLetter = pt.get_child("trial.actual_goal").data();
	target = getTargetLetter(targetLetter);
	int n = 1;
	int frame = 1;
	float lookAtCounter = 0;
	float lookAtPercentage = 0;
	float length = to - from;

	// initialize the csv files
	lookAtTargetOne.open (pt.get_child("trial.id").data() + "LookAtTarget.csv");

	lookAtTargetLetterA.open("ALookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterB.open("BLookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterC.open("CLookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterD.open("DLookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterE.open("ELookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterF.open("FLookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterG.open("GLookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);
	lookAtTargetLetterH.open("HLookAtTarget.csv",std::fstream::in | std::fstream::out | std::fstream::app);

	lookAtTargetAll.open ("LookAtTargetAllFiles.csv", std::fstream::in | std::fstream::out | std::fstream::app);

	lookAtTargetperWalker.open (walker + "LookAtTarget.csv", std::fstream::in | std::fstream::out | std::fstream::app);

	if (pt.get_child("trial.task").data() == "?")
		lookAtTargetOne << "target: ? - " << targetLetter << std::endl;
	else
		lookAtTargetOne << "target: " << targetLetter << std::endl;

	lookAtTargetOne << "frame; degrees" << std::endl;
	
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
					crossproduct = getCrossProduct(walkerVecX, walkerVecY, 0, targetVecX, targetVecY, 0);
					angle = std::acosf(dotProduct / 1); // length is 1, they are unit vectors
					
					angleDeg = angle*(180/M_PI); //  to degrees

					// lookAtTarget yes/no
					if (dotProduct > 0 )
						lookAtCounter++;
					if (crossproduct[2] >= 0)
					{
						if (targetLetter == "a")
							lookAtTargetLetterA << angleDeg << std::endl;
						else if (targetLetter == "b")
							lookAtTargetLetterB << angleDeg << std::endl;
						else if (targetLetter == "c")
							lookAtTargetLetterC << angleDeg << std::endl;
						else if (targetLetter == "d")
							lookAtTargetLetterD << angleDeg << std::endl;
						else if (targetLetter == "e")
							lookAtTargetLetterE << angleDeg << std::endl;
						else if (targetLetter == "f")
							lookAtTargetLetterF << angleDeg << std::endl;
						else if (targetLetter == "g")
							lookAtTargetLetterG << angleDeg << std::endl;
						else if (targetLetter == "h")
							lookAtTargetLetterH << angleDeg << std::endl;

						lookAtTargetOne << (frame) <<";" << angleDeg << std::endl;
						lookAtTargetperWalker << angleDeg << std::endl;
						lookAtTargetAll << angleDeg << std::endl;
					}
					else
					{
						if (targetLetter == "a")
							lookAtTargetLetterA << -angleDeg << std::endl;
						else if (targetLetter == "b")
							lookAtTargetLetterB << -angleDeg << std::endl;
						else if (targetLetter == "c")
							lookAtTargetLetterC << -angleDeg << std::endl;
						else if (targetLetter == "d")
							lookAtTargetLetterD << -angleDeg << std::endl;
						else if (targetLetter == "e")
							lookAtTargetLetterE << -angleDeg << std::endl;
						else if (targetLetter == "f")
							lookAtTargetLetterF << -angleDeg << std::endl;
						else if (targetLetter == "g")
							lookAtTargetLetterG << -angleDeg << std::endl;
						else if (targetLetter == "h")
							lookAtTargetLetterH << -angleDeg << std::endl;

						lookAtTargetOne << (frame) <<";" << -angleDeg << std::endl;
						lookAtTargetperWalker << -angleDeg << std::endl;
						lookAtTargetAll << -angleDeg << std::endl;
					}
				}
				n = 1;
				frame++;
				
			}
		}
	}
	
	lookAtTargetOne.close();
	lookAtTargetLetterA.close();
	lookAtTargetLetterB.close();
	lookAtTargetLetterC.close();
	lookAtTargetLetterD.close();
	lookAtTargetLetterE.close();
	lookAtTargetLetterF.close();
	lookAtTargetLetterG.close();
	lookAtTargetLetterH.close();
	lookAtTargetperWalker.close();
	lookAtTargetAll.close();

	lookAtPercentage =  (lookAtCounter/length) * 100.0;
	return lookAtPercentage;
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
	std::ofstream lookAtTargetPerc, AngularLinearVelocity;
	float lookAtTargetPercentage = 0;

    lookAtTargetPerc.open ("LookAtTarget.csv");
	lookAtTargetPerc << "Trial; target; lookedAtTarget; frames" << std::endl;

	directory_iterator end_iter;
	for (directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
	{
		if (dir_itr->path().filename().extension().string() != ".gz")
		{
			std::cout<<"File " << dir_itr->path().filename().string() << " is not a .gz file";
			continue;
		}

		boost::property_tree::ptree pt;
		int from, to, total;
		try 
		{
			pt = loadTree(dir_itr->path().string());
			from = std::stoi(pt.get_child("trial.dense_movement_start").data());// start dense movement
			to = std::stoi(pt.get_child("trial.dense_movement_end").data()); // end dense movement
			total = to - from;
			lookAtTargetPercentage = getLookAtTarget(pt, from, to);
			if (pt.get_child("trial.task").data() == "?")
				lookAtTargetPerc << pt.get_child("trial.id").data() << ";" << "? - " << pt.get_child("trial.actual_goal").data() << ";" << lookAtTargetPercentage << "%" << ";" << total << std::endl;
			else
				lookAtTargetPerc << pt.get_child("trial.id").data() << ";" << pt.get_child("trial.actual_goal").data() << ";" << lookAtTargetPercentage << "%" << ";" << total << std::endl;
			//getBothVelocities(pt, from, to);
		}
		catch (UuIcsC3d::OpenError const &err) 
		{
			cerr<<"An error occurred while processing file "<< err.filename();
			return;
		}
	}
	lookAtTargetPerc.close();
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

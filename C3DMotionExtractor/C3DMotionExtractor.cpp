// C3DMotionExtractor.cpp : Defines the entry point for the console application.
//

/**
  * \file
  * \brief A commented example file; a good starting point learning the C3d library.
  */

/*
A C3D file contains a number of frames (time steps) and labels.
In every frame a label can either be not present (invalid), measured
(detected by hardware) or generated (e.g., filled in by interpolation).
This program counts those three possibilities and displays the totals for
every label.
*/

#include "stdafx.h"
#include "uuc3d.hpp"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>

using std::cerr;
using std::vector;
using UuIcsC3d::SpacePaddedString;
using UuIcsC3d::DataPoint3d;
using namespace boost::filesystem;


vector<float> calcMotion(vector<float> point1, vector<float> point2, vector<float> point3)
{
	vector<float> motion(3);

	float center23X = (point2[0] + point3[0]) / 2;
	float center23Y = (point2[1] + point3[1]) / 2;
	float centerX = (point1[0] + center23X) / 2;
	float centerY = (point1[1] + center23Y) / 2;
	float rot = atan2f(centerY, centerX);

	motion[0] = centerX;
	motion[1] = centerY;
	motion[2] = rot;
	
	return motion;
}

void do_work(std::string filename)
{
    // Open the file and read the header information
    // Will throw OpenError if reading is not possible.
    UuIcsC3d::C3dFileInfo fi(filename);
    // Every frame contains the same number of points.
    int ppf = fi.points_per_frame(); 
    if (ppf <= 0)
	{
		return;
	}
    int fc = fi.frame_count();
	std::cout << "frames: " << fc << "\n";

	// create the tree to parse to json format. 
	boost::property_tree::ptree pt;
	boost::property_tree::ptree subjects;
	boost::property_tree::ptree subject1, subject2, subject3;
	boost::property_tree::ptree motions1, motions2, motions3;
	boost::property_tree::ptree m, mX, mY, mRot;

    // Get the labels. Not every point needs to have a label name.
    // We add dummy labels if necessary to supply a name for every point.
    vector<SpacePaddedString> labels(fi.point_labels());
    for (int i=labels.size(); i<ppf; ++i) 
	{
		labels.push_back(SpacePaddedString("<no label>"));
    }

    // Declare the vectors that maintain the count for each point (label).
    vector<int> invalid(ppf, 0), generated(ppf, 0), measured(ppf, 0);

    // Open the C3D file
    std::auto_ptr<UuIcsC3d::C3dFile> filep(fi.open());

    // Read every frame in variable frame_data and process it
    UuIcsC3d::FrameData frame_data;
	std::size_t found;

	int counter = 0;
	vector<float> motion(3);
	bool test = true;

    for (int i=0; i<fc; ++i)
	{
		filep->get_frame_data(frame_data, i);
		vector<float> point1(2), point2(2), point3(2);

		// For every point in the frame record its type
		for (int j=0; j<ppf; ++j) 
		{
			DataPoint3d const &dp=frame_data.points[j];
			found = labels[j].stripped().find("1");
			if (found == 2)
			{
				point1[0] = dp.x();
				point1[1] = dp.y();
				counter++;
			}
			found = labels[j].stripped().find("2");
			if (found == 2)
			{
				point2[0] = dp.x();
				point2[1] = dp.y();
				counter++;
			}
			found = labels[j].stripped().find("3");
			if (found == 2)
			{
				point3[0] = dp.x();
				point3[1] = dp.y();
				counter++;
			}

			if (counter == 3)
			{
				motion = calcMotion(point1, point2, point3);

				mX.put("", motion[0]);
				mY.put("", motion[1]);
				mRot.put("", motion[2]);

				m.push_back(std::make_pair("", mX));
				m.push_back(std::make_pair("", mY));
				m.push_back(std::make_pair("", mRot));
				if (test)
				{
					//std::cout<<"motion: " << motion[0] << " " << motion[1] << " " << motion[2] << "\n";
				}
				if (labels[j].stripped().find("LH") == 0)
				{
					motions1.push_back(std::make_pair("", m));
				}
				else if (labels[j].stripped().find("RH") == 0)
				{
					motions2.push_back(std::make_pair("", m));
				}
				else if (labels[j].stripped().find("LB") == 0)
				{
					motions3.push_back(std::make_pair("", m));
				}

				mX.clear();
				mY.clear();
				mRot.clear();
				m.clear();
				counter = 0;
			}
			switch (dp.status()) {
				case DataPoint3d::Invalid:
					++invalid[j];
					break;
				case DataPoint3d::Generated:
					++generated[j];
					break;
				case DataPoint3d::Measured:
					++measured[j];
					break;
			}
		}
    }
    // report the counts for every label
	std::cout << "labels: \n";
    for (int i=0; i<ppf; ++i) 
	{
		std::cout <<labels[i].stripped()<<"\n";
    }

	subject1.put("chest_width", 0.54);
	subject1.put("chest_thickness",0.32);
	subject1.put("role","passive");
	subject1.put("gender","m");
	subject2.put("chest_width", 0.62);
	subject2.put("chest_thickness",0.41);
	subject2.put("role","active");
	subject2.put("gender","m");
	subject3.put("chest_width", 0.56);
	subject3.put("chest_thickness",0.34);
	subject3.put("role","passive");
	subject3.put("gender","f");

	subject1.add_child("motion", motions1);
	subject2.add_child("motion", motions2);
	subject3.add_child("motion", motions3);
	subjects.push_back(std::make_pair("LH", subject1));
	subjects.push_back(std::make_pair("RH", subject2));
	subjects.push_back(std::make_pair("LB", subject3));
	pt.add_child("Subjects",subjects);

	std::string file = filename.substr(0,filename.size()-4);
	std::string name = file + ".json";
	write_json( name, pt);
}

void open_folder(std:: string folder)
{
	path p(folder);
	if (exists(p))
	{
		if (is_directory(p))
		{
			directory_iterator end_iter;
			for (directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
			{
				if (dir_itr->path().filename().extension().string() == ".c3d")
				{
					try 
					{
						do_work(dir_itr->path().string());
					}
					catch (UuIcsC3d::OpenError const &err) 
					{
						cerr<<"An error occurred while processing file "<< err.filename();
						return;
					}
				}
					
			}
		}
		else
		{
			std::cout<< p << "is not a folder";
			return;
		}
	} 
	else 
	{
		std::cout<< p << "does not exist";
		return;
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


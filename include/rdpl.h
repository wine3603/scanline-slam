/*******************************************************
* Copyright (c) 2016 1 4 the Univ. of Tokyo || YNL
*
* @file rdpl.cpp
* @ brief relization of Ramen Douglas Peucker Line Simplification algorithm
* @ author Tianwei Zhang
**********************************************************/
#ifndef RDPL_H
#define RDPL_H
#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <region_grow.h>

using namespace std;
using namespace region_growing;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const float bad_point = std::numeric_limits<float>::quiet_NaN();
class rdpl {
// make a stack data structure to store RDP iterative elements
public:
//template <typename PointT>  
float PointLineDistance(PointT&  point, PointT& start, PointT& end) {
	//if (start == end) {
		//return Vector2.Distance(point, start);
	//	return 0;	
	//	}
	
	float n = fabs((end.x - start.x) * (start.y - point.y) - (start.x - point.x) * (end.y - start.y));
	float d = sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
	
	return n / d;
	}

//struct StackElement
//	{
//	  PointT point;
//	  size_t index;
//	};
//	std::vector< StackElement> rdp_stack;
	split_line rdp_stack;
	split_point StackElement;
//private:
// rdp function , with a given threshod epsilon, return 0 or 1 (bigger dis)

int rdp_implimentation(       PointCloudT& input,
                              split_line& output,//PointCloudT& output,
                                  double e )
{
    output.resize( 0 );

    if ( input.empty( ) )
    {
        return 0;
    }

    // Clean up
    rdp_stack.resize( 0 );
//std::cout<<"input.."<<input.size()<<"..output"<<output.size()<<"...epsilon: "<<e<<std::endl;
/*   if (rdp_stack.reserve( input.size( ) ) <= 0
        || output.reserve( input.size( ) ) <= 0 )
    {
        // Failed to allocate memory
        return -1;
    }
*/
    PointT anchor = input.points[0];
    size_t index_of_anchor = 0;
    size_t length_of_anchor = 0;
    PointT floater = input.points[input.size()];
    size_t index_of_floater = input.size( ) - 1;
    size_t length_of_floater = 0;

    // Add the first point in the poly to the result
	split_point temp_point_n;
	temp_point_n.end_point = anchor;
	temp_point_n.index = 0;//TODO
	temp_point_n.length = 0;
    output.push_back(temp_point_n );

    split_point stack_element =
    {
        floater,
        index_of_floater,
	length_of_floater
    };

    rdp_stack.push_back( stack_element );

 while ( !rdp_stack.empty( ) )
    {
        double max_squared_distance = 0.0;
        PointT farthest = anchor;
        size_t index_of_farthest = index_of_anchor;

        // Find point furthest from line defined by anchor and floater
        // function depends on your projection/dimension
       // DistanceHelper distance_helper( anchor, floater );

        for ( size_t i = index_of_anchor + 1; i < index_of_floater; ++i )
        {

            const double squared_distance = PointLineDistance (input.points[i], anchor, floater);// distance_helper.squared_distance_to( input[ i ] );
            if ( squared_distance > max_squared_distance )
            {
                max_squared_distance = squared_distance;
                farthest = input.points[i];
                index_of_farthest = i;
            }
        }

/////adaptive e	
	float range = farthest.intensity;
	if( 0.1  <= range && range  < 1.0)
	e = 0.05;
	else if ( range < 5.0)
	e = 0.05;
	else if (range< 10)
	e = 0.05;
	else e = 0.1;
 
        // Furthest point nearer than tolerance?
        if ( max_squared_distance <= e )
        {
            output.push_back( rdp_stack.back( ) );
            rdp_stack.pop_back( );
            anchor = floater;
            index_of_anchor = index_of_floater;
            if ( !rdp_stack.empty( ) )
            {
                floater = rdp_stack.back( ).end_point;
                index_of_floater = rdp_stack.back( ).index;
            }
        }
        else
        {
            floater = farthest;
            index_of_floater = index_of_farthest;
            stack_element.end_point = floater;
            stack_element.index = index_of_floater;
            rdp_stack.push_back( stack_element );
        }
    }
//std::cout<<"input.."<<input.size()<<"..output"<<output.size()<<"...epsilon: "<<e<<std::endl;
    // Success
    return 0;
};
};
#endif

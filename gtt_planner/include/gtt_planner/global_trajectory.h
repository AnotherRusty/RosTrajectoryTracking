/*********************************************************************
 *
 *  Copyright (c) 2019, 2030, Pibot Technology (Suzhou) Co., Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Pibot Technology (Suzhou) Co., Ltd. nor the
 *     names of its contributors may be used to endorse or promote 
 *     products derived from this software without specific prior 
 *     written permission.
 *
 * Author: Danny Zhu @Pibot
 *         
 *********************************************************************/

#ifndef _GLOBAL_TRAJECTORY_H
#define _GLOBAL_TRAJECTORY_H

#include <ros/ros.h>

namespace gtt_planner {

//  @brief A class that manages the global trajectory to track.
class GlobalTrajectory
{
    public:
        GlobalTrajectory();
        ~GlobalTrajectory();

        /**
         * @brief  Load the global trajectory from yaml.
         * @param file  The location of the yaml file.
         */
        bool loadPath(const std::string& file);
        
        /**
         * @brief  Get the full global trajectory.
         */
        void getFullPath(std::vector<std::pair<double, double> >& path);

        /**
         * @brief  Get the trimmed global trajectory to track.
         * @param start  The start point.
         */
        void getTrimmedPath(const std::pair<double, double>& start, std::vector<std::pair<double, double> >& path);

    private:
        /**
         * @brief  Search for the nearest point on the given trajectory.
         * @param x X position of the start point.
         * @param x Y position of the start point.
         * @return  The index of the point in the global trajectory.
         */
        int getNearestPoint(const double& x, const double& y);

        std::vector<std::pair<double, double> > gt_;    // glocal trajectory

};

} // end namespace gtt_planner

#endif
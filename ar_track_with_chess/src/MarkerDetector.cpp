/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#include "ar_track_alvar/MarkerDetector.h"
#include <ros/ros.h>
#include "vision/ChessPiece.h"
#include "vision/ChessPoint.h"
#include "vision/ChessVector.h"
#include "vision/ChessPiecesVector.h"
#include "vision/ChessInfoVector.h"
#include "vision/ChessBoard.h"

template class ALVAR_EXPORT alvar::MarkerDetector<alvar::Marker>;
template class ALVAR_EXPORT alvar::MarkerDetector<alvar::MarkerData>;
template class ALVAR_EXPORT alvar::MarkerDetector<alvar::MarkerArtoolkit>;

using namespace std;

namespace alvar {
	MarkerDetectorImpl::MarkerDetectorImpl() {
		SetMarkerSize();
		SetOptions();
		labeling = NULL;
	}

	MarkerDetectorImpl::~MarkerDetectorImpl() {
		if (labeling) delete labeling;
	}

	void MarkerDetectorImpl::TrackMarkersReset() {
		_track_markers_clear();
	}

	void MarkerDetectorImpl::TrackMarkerAdd(int id, PointDouble corners[4]) {
    Marker *mn = new_M(edge_length, res, margin);
		if (map_edge_length.find(id) != map_edge_length.end()) {
			mn->SetMarkerSize(map_edge_length[id], res, margin);
		}

		mn->SetId(id);
		mn->marker_corners_img.clear();
		mn->marker_corners_img.push_back(corners[0]);
		mn->marker_corners_img.push_back(corners[1]);
		mn->marker_corners_img.push_back(corners[2]);
		mn->marker_corners_img.push_back(corners[3]);
		_track_markers_push_back(mn);
    delete mn;
	}

	void MarkerDetectorImpl::SetMarkerSize(double _edge_length, int _res, double _margin) {
		edge_length = _edge_length;
		res = _res;
		margin = _margin;
		map_edge_length.clear(); // TODO: Should we clear these here?
    }

	void MarkerDetectorImpl::Initialize_Chess2dArray(){
		vision::ChessPoint pt;		
		vision::ChessPiece pc;
		chessPiecesArea_2dcoordinates.p_vector.clear();
		chess_2dcoordinates.p_vector.clear();
		for(int i=0;i<81;i++){
			pt.x=0;
			pt.y=0;
			pt.state="empty";
			chess_2dcoordinates.p_vector.push_back(pt);
			if(i<64){
				pc.a.x=0; pc.a.y=0; pc.a.state="empty";
				pc.b.x=0; pc.b.y=0; pc.b.state="empty";
				pc.c.x=0; pc.c.y=0; pc.c.state="empty";
				pc.d.x=0; pc.d.y=0; pc.d.state="empty";
				pc.e.x=0; pc.e.y=0; pc.e.state="empty";
				pc.f.x=0; pc.f.y=0; pc.f.state="empty";
				pc.g.x=0; pc.g.y=0; pc.g.state="empty";
				pc.h.x=0; pc.h.y=0; pc.h.state="empty";
				pc.category="empty";
				chessPiecesArea_2dcoordinates.p_vector.push_back(pc);
			}
		}
	} 

/*	void MarkerDetectorImpl::Update_Chess2dArray(){
		for(int i=0;i<81;i++){
			chess_2dcoordinates.p_vector[i].x=480;
			chess_2dcoordinates.p_vector[i].y=640;
		}
		//ROS_INFO("updated");
	} */

	void MarkerDetectorImpl::SetMarkerSizeForId(unsigned long id, double _edge_length) {
		map_edge_length[id] = _edge_length;
	}

	void MarkerDetectorImpl::SetOptions(bool _detect_pose_grayscale) {
		detect_pose_grayscale = _detect_pose_grayscale;
	}

	vision::ChessInfoVector  MarkerDetectorImpl::DetectChess(IplImage *image,
			   Camera *cam,
			   vision::ChessBoard game_,
			   bool track,
			   bool visualize,
			   double max_new_marker_error,
			   double max_track_error,
			   LabelingMethod labeling_method,
			   bool update_pose)
	{
		assert(image->origin == 0); // Currently only top-left origin supported
		double error=-1;

		// Swap marker tables
		_swap_marker_tables();
		_markers_clear();
		switch(labeling_method)
		{
			case CVSEQ :
		
				if(!labeling)
					labeling = new LabelingCvSeq();
				((LabelingCvSeq*)labeling)->SetOptions(detect_pose_grayscale);
				break;
		}

		labeling->SetCamera(cam);
		labeling->LabelSquares(image, visualize);
		vector<vector<PointDouble> >& blob_corners = labeling->blob_corners;
		IplImage* gray = labeling->gray;

		int orientation;

		// When tracking we find the best matching blob and test if it is near enough?
		if (track) {
			for (size_t ii=0; ii<_track_markers_size(); ii++) {
				Marker *mn = _track_markers_at(ii);

				if (mn->GetError(Marker::DECODE_ERROR|Marker::MARGIN_ERROR) > 0) continue; // We track only perfectly decoded markers
				int track_i=-1;
				int track_orientation=0;
				double track_error=1e200;
				for(unsigned i = 0; i < blob_corners.size()/*blobs_ret.size()*/; ++i) {
					if (blob_corners[i].empty()) continue;
					mn->CompareCorners(blob_corners[i], &orientation, &error);
					if (error < track_error) {
						track_i = i;
						track_orientation = orientation;
						track_error = error;
					}
				}
				if (track_error <= max_track_error) {
					mn->SetError(Marker::DECODE_ERROR, 0);
					mn->SetError(Marker::MARGIN_ERROR, 0);
					mn->SetError(Marker::TRACK_ERROR, track_error);
                    mn->UpdateContent(blob_corners[track_i], gray, cam);    //Maybe should only do this when kinect is being used? Don't think it hurts anything...
					mn->UpdatePose(blob_corners[track_i], cam, track_orientation, update_pose);
					_markers_push_back(mn);
					blob_corners[track_i].clear(); // We don't want to handle this again...
					if (visualize){
						chess_2dcoordinates=mn->VisualizeChess(image, cam, game_, chess_2dcoordinates,CV_RGB(0,255,0));
						if(game_.chessSquare.size()!=0){ //unless there are pawn on the table..
							chessPiecesArea_2dcoordinates=mn->VisualizeChessPawns(image, cam, game_, chessPiecesArea_2dcoordinates,CV_RGB(0,255,0));
						}
						//mn->Update_Chess2dArray();
						ROS_INFO("coordinates'  vector size: %d",chess_2dcoordinates.p_vector.size());
						ROS_INFO("pieces' areas vector size: %d",chessPiecesArea_2dcoordinates.p_vector.size());
					}
				}
			}
		}

		// Now we go through the rest of the blobs -- in case there are new markers... not 100%sure
		for(size_t i = 0; i < blob_corners.size(); ++i)
		{
			if (blob_corners[i].empty()) continue;

			Marker *mn = new_M(edge_length, res, margin);
			bool ub = mn->UpdateContent(blob_corners[i], gray, cam);
            bool db = mn->DecodeContent(&orientation); 
			if (ub && db &&
				(mn->GetError(Marker::MARGIN_ERROR | Marker::DECODE_ERROR) <= max_new_marker_error))
			{
				if (map_edge_length.find(mn->GetId()) != map_edge_length.end()) {
					mn->SetMarkerSize(map_edge_length[mn->GetId()], res, margin);
				}
				mn->UpdatePose(blob_corners[i], cam, orientation, update_pose);
                mn->ros_orientation = orientation;
				_markers_push_back(mn); 
				if (visualize){
					chess_2dcoordinates=mn->VisualizeChess(image, cam, game_, chess_2dcoordinates,CV_RGB(0,0,255));					
					if(game_.chessSquare.size()!=0){ //unless there are pawn on the table..
						chessPiecesArea_2dcoordinates=mn->VisualizeChessPawns(image, cam, game_, chessPiecesArea_2dcoordinates,CV_RGB(0,255,0));
					}
					//mn->Update_Chess2dArray();
					ROS_INFO("coordinates'  vector size: %d",chess_2dcoordinates.p_vector.size());
					ROS_INFO("pieces' areas vector size: %d",chessPiecesArea_2dcoordinates.p_vector.size());
				}
			}
			
			delete mn;
		}

		//ROS_INFO("end");
		vision::ChessInfoVector info_vector;
		info_vector.pieces.push_back(chessPiecesArea_2dcoordinates);
		info_vector.knob_points.push_back(chess_2dcoordinates);
		return info_vector;
	}

	int MarkerDetectorImpl::Detect(IplImage *image,
			   Camera *cam,
			   bool track,
			   bool visualize,
			   double max_new_marker_error,
			   double max_track_error,
			   LabelingMethod labeling_method,
			   bool update_pose)
	{
		assert(image->origin == 0); // Currently only top-left origin supported
		double error=-1;

		// Swap marker tables
		_swap_marker_tables();
		_markers_clear();
		switch(labeling_method)
		{
			case CVSEQ :
		
				if(!labeling)
					labeling = new LabelingCvSeq();
				((LabelingCvSeq*)labeling)->SetOptions(detect_pose_grayscale);
				break;
		}

		labeling->SetCamera(cam);
		labeling->LabelSquares(image, visualize);
		vector<vector<PointDouble> >& blob_corners = labeling->blob_corners;
		IplImage* gray = labeling->gray;

		int orientation;

		// When tracking we find the best matching blob and test if it is near enough?
		if (track) {
			for (size_t ii=0; ii<_track_markers_size(); ii++) {
				Marker *mn = _track_markers_at(ii);

				if (mn->GetError(Marker::DECODE_ERROR|Marker::MARGIN_ERROR) > 0) continue; // We track only perfectly decoded markers
				int track_i=-1;
				int track_orientation=0;
				double track_error=1e200;
				for(unsigned i = 0; i < blob_corners.size()/*blobs_ret.size()*/; ++i) {
					if (blob_corners[i].empty()) continue;
					mn->CompareCorners(blob_corners[i], &orientation, &error);
					if (error < track_error) {
						track_i = i;
						track_orientation = orientation;
						track_error = error;
					}
				}
				if (track_error <= max_track_error) {
					mn->SetError(Marker::DECODE_ERROR, 0);
					mn->SetError(Marker::MARGIN_ERROR, 0);
					mn->SetError(Marker::TRACK_ERROR, track_error);
                    mn->UpdateContent(blob_corners[track_i], gray, cam);    //Maybe should only do this when kinect is being used? Don't think it hurts anything...
					mn->UpdatePose(blob_corners[track_i], cam, track_orientation, update_pose);
					_markers_push_back(mn);
					blob_corners[track_i].clear(); // We don't want to handle this again...
					if (visualize) mn->Visualize(image, cam, CV_RGB(255,0,0));
				}
			}
		}

		// Now we go through the rest of the blobs -- in case there are new markers... not 100%sure
		for(size_t i = 0; i < blob_corners.size(); ++i)
		{
			if (blob_corners[i].empty()) continue;

			Marker *mn = new_M(edge_length, res, margin);
			bool ub = mn->UpdateContent(blob_corners[i], gray, cam);
            bool db = mn->DecodeContent(&orientation); 
			if (ub && db &&
				(mn->GetError(Marker::MARGIN_ERROR | Marker::DECODE_ERROR) <= max_new_marker_error))
			{
				if (map_edge_length.find(mn->GetId()) != map_edge_length.end()) {
					mn->SetMarkerSize(map_edge_length[mn->GetId()], res, margin);
				}
				mn->UpdatePose(blob_corners[i], cam, orientation, update_pose);
                mn->ros_orientation = orientation;
				_markers_push_back(mn);
 
				if (visualize) mn->Visualize(image, cam, CV_RGB(255,255,0));
			}

			delete mn;
		}

		return (int) _markers_size();
	}

	int MarkerDetectorImpl::DetectAdditional(IplImage *image, Camera *cam, bool visualize, double max_track_error)
	{
		assert(image->origin == 0); // Currently only top-left origin supported
		if(!labeling) return -1;
		double error=-1;
		int orientation;
		int count=0;
		vector<vector<PointDouble> >& blob_corners = labeling->blob_corners;

		for (size_t ii=0; ii<_track_markers_size(); ii++) {
			Marker *mn = _track_markers_at(ii);
			if (mn->GetError(Marker::DECODE_ERROR|Marker::MARGIN_ERROR) > 0) continue; // We track only perfectly decoded markers
			int track_i=-1;
			int track_orientation=0;
			double track_error=1e200;
			for(unsigned i = 0; i < blob_corners.size(); ++i) {
				if (blob_corners[i].empty()) continue;
				mn->CompareCorners(blob_corners[i], &orientation, &error);
				if (error < track_error) {
					track_i = i;
					track_orientation = orientation;
					track_error = error;
				}
			}
			if (track_error <= max_track_error) {
				mn->SetError(Marker::DECODE_ERROR, 0);
				mn->SetError(Marker::MARGIN_ERROR, 0);
				mn->SetError(Marker::TRACK_ERROR, track_error);
				mn->UpdatePose(blob_corners[track_i], cam, track_orientation);
				_markers_push_back(mn);
				count++;
				blob_corners[track_i].clear(); // We don't want to handle this again...

				if (visualize) {
					mn->Visualize(image, cam, CV_RGB(0,255,255));
				}
			}
		}
		return count;
	}

}

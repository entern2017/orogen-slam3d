#ifndef SLAM3D_ROCK_ODOMETRY_HPP
#define SLAM3D_ROCK_ODOMETRY_HPP

#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Solver.hpp>
#include <transformer/Transformer.hpp>

namespace slam3d
{
	class RockOdometry : public PoseSensor
	{
	public:
		/**
		 * @brief Pose sensor that uses the Rock transformer
		 * @param name
		 * @param graph
		 * @param s
		 * @param logger
		 * @param tf
		 * @param interpolate set to true when align_port is used
		 */
		RockOdometry(const std::string& name, Graph* graph, Solver* s, Logger* logger,
			transformer::Transformation& tf, bool interpolate);
		~RockOdometry();

		void setGravityReference(const Direction& ref) {mGravityReference = ref;}
		void handleNewVertex(IdType vertex);
		
		Transform getPose(timeval stamp);
		Transform getPose(base::Time t);
		Covariance<6> calculateCovariance(const Transform &tf);

	private:
		transformer::Transformation& mTransformation;
		Transform mLastOdometricPose;
		IdType mLastVertex;
		Direction mGravityReference;
		Solver* mSolver;
		bool mInterpolate;
	};
}

#endif

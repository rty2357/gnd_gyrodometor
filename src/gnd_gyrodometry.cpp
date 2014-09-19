/**
 * @file gnd_gyro_odometry/src/main.cpp
 *
 * @brief Test Program
 **/

#include "gnd/gnd-multi-platform.h"

#include <stdio.h>

#include "gnd/gnd_gyrodometry.hpp"
#include "gnd/gnd_gyrodometry_conf.hpp"

#include "gnd_geometry2d_msgs/msg_pose2d_stamped.h"
#include "gnd_geometry2d_msgs/msg_velocity2d_stamped.h"

#include "vectornav/sensors.h"

#include "gnd/gnd_rosmsg_reader.hpp"
#include "gnd/gnd_rosutil.hpp"

#include "gnd/gnd-multi-math.h"
#include "gnd/gnd-util.h"

#include "ros/ros.h"

typedef gnd_geometry2d_msgs::msg_pose2d_stamped				msg_pose2d_t;
typedef gnd_geometry2d_msgs::msg_velocity2d_stamped			msg_vel2d_t;
typedef vectornav::sensors									msg_imu_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_vel2d_t>	msgreader_vel2d_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_imu_t>		msgreader_imu_t;

int main(int argc, char *argv[]) {
	gnd::gyrodometry::node_config			node_config;
	{ // ---> read configuration
		if( argc > 1 ) {
			fprintf(stdout, " => read configuration file\n");
			if( gnd::gyrodometry::fread_node_config( argv[1], &node_config ) < 0 ){
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read configuration file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( gnd::gyrodometry::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}
	} // <--- read configuration

	{ // ---> init ros
		fprintf(stdout, "\n");
		fprintf(stdout, " => call ros::init()\n");

		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}

		fprintf(stdout, "    ... ok, node name is \"%s\"\n", node_config.node_name.value);
	} // <--- init ros


	// ---> ros communication object
	ros::NodeHandle		nodehandle;				// node handle
	ros::Publisher		pub_gyrodo;				// gyrodometory publisher
	msg_pose2d_t		msg_gyrodo;				// gydorodometry message

	ros::Subscriber		subsc_vel2d;			// velocity by gyrodometry publisher
	msg_vel2d_t			msg_vel2d;				// message of velocity by gyrodometry
	msgreader_vel2d_t	msgreader_vel2d;		// message reader of velocity by gyrodometry

	ros::Subscriber		subsc_imu;				// imu subscriber
	msg_imu_t			msg_imu;				// imu message
	msgreader_imu_t		msgreader_imu;			// imu reader
	// <--- ros communication object

	// debug
	FILE *fp_textlog = 0;


	// ---> initialize
	if( ros::ok() ){
		int phase = 0;
		ros::Time time_nodestart = ros::Time::now();

		fprintf(stdout, "---------- initialize ----------\n");


		if( ros::ok() ) { // ---> show initialize phase task
			fprintf(stdout, " initialization task\n");
			fprintf(stdout, "   %d. make gyrodometry publisher\n", ++phase);
			fprintf(stdout, "   %d. make vel2d subscriber\n", ++phase);
			fprintf(stdout, "   %d. make imu subscriber\n", ++phase);
			if( node_config.gyrodometry_log.value[0] ) {
				fprintf(stdout, "   %d. create text log file\n", ++phase);
			}
			fprintf(stdout, "\n\n");
		} // <--- show initialize phase task
		phase = 0;

		// ---> initialize gyrodometry publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make gyrodometry publisher\n", ++phase);
			if( !node_config.topic_name_gyrodom.value[0] ) {
				fprintf(stdout, "   ... Error: gyrodometry topic name is null, you needs to specify the name via config item \"%s\"\n", node_config.topic_name_gyrodom.item);
				ros::shutdown();
			}
			else {
				pub_gyrodo = nodehandle.advertise<gnd_geometry2d_msgs::msg_pose2d_stamped>(node_config.topic_name_gyrodom.value, 1000);

				msg_gyrodo.x = 0;
				msg_gyrodo.y = 0;
				msg_gyrodo.theta = 0;
				msg_gyrodo.header.stamp = time_nodestart;
				msg_gyrodo.header.seq = 0;
				msg_gyrodo.header.frame_id;

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize gyrodometry publisher

		// ---> initialize vel2d subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make velocity subscriber\n", ++phase);
			if( !node_config.topic_name_vel2d.value[0] ) {
				fprintf(stdout, "   ... Error: velocity topic name is null, you needs to specify the name via config item \"%s\"\n", node_config.topic_name_vel2d.item);
				ros::shutdown();
			}
			else {
				// subscribe
				msgreader_vel2d.allocate(1000);
				subsc_vel2d = nodehandle.subscribe(node_config.topic_name_vel2d.value, 1000,
						&msgreader_vel2d_t::rosmsg_read,
						msgreader_vel2d.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize vel2d subscriber

		// ---> initialize imu subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make imu publisher\n", ++phase);
			if( !node_config.topic_name_vel2d.value[0] ) {
				fprintf(stdout, "   ... Error: imu topic name is null, you needs to specify the name via config item \"%s\"\n", node_config.topic_name_imu.item);
				ros::shutdown();
			}
			else {
				// subscribe
				msgreader_imu.allocate(1000);
				subsc_imu = nodehandle.subscribe(node_config.topic_name_imu.value, 1000,
						&msgreader_imu_t::rosmsg_read,
						msgreader_imu.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize vel2d subscriber

		if( ros::ok() && node_config.gyrodometry_log.value[0] ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. create text log file\n", ++phase );
			fprintf(stdout, "    ... try to open \"%s\"\n", node_config.gyrodometry_log.value );

			if( !(fp_textlog = fopen( node_config.gyrodometry_log.value, "w"))  ) {
				fprintf(stdout, "    ... error: fail to open \"%s\"\n", node_config.gyrodometry_log.value );
				ros::shutdown();
			}
			else {
				fprintf(fp_textlog, "# [1. time] [2. x] [3. y] [4. theta] [5. offset]\n" );
				fprintf(stdout, "    ... ok\n");
			}
		}

	} // <--- initialize


	// ---> operation
	if( ros::ok() ) {
		double offset_rate;
		double time_latestvel2d = 0;
		double time_latestimu = 0;

		double time_start;
		double time_vehiclestop;
		double time_current;
		double time_gyrodom;
		double time_display;

		const double cycle_display = gnd_sec2time(1.0);

		double diff_time = 0;

		ros::Rate loop_rate(1000);

		uint32_t seq_gyrodo_prevdisp = msg_gyrodo.header.seq;
		uint32_t seq_vel2d_prevdisp = msg_vel2d.header.seq;
		uint32_t seq_imu_prevdisp = msg_imu.header.seq;
		int nline_display = 0;

		{ // ---> gyro sensor offset initialize
			offset_rate = node_config.offset_calibration_default.value;
		} // <--- gyro sensor offset initialize

		{ // ---> time initialize
			time_current = ros::Time::now().toSec();
			time_start = time_current;
			time_gyrodom = time_start;
			time_vehiclestop = time_start;
			time_display = time_start;
		} // <--- time initialize

		// ---> main loop
		while( ros::ok() ){
			// sleep
			loop_rate.sleep();
			// subscribe
			ros::spinOnce();

			// save time
			time_current = ros::Time::now().toSec();

			// ---> calculate 2d pose
			if( time_current > time_gyrodom ) {
				// get velocity2d massage
				if( msgreader_vel2d.copy_next( &msg_vel2d, msg_vel2d.header.seq) < 0) {
					// no data
				}
				else {
					time_latestvel2d = time_current;
				}

				// get imu massage
				if( msgreader_imu.copy_at_time( &msg_imu, msg_vel2d.header.stamp.toSec() ) < 0) {
					// no data
				}
				// ---> pose calculation
				else if( time_current - time_latestvel2d < 0.3  ) {
					double cosv, sinv;
					double rate;

					cosv = cos( msg_gyrodo.theta );
					sinv = sin( msg_gyrodo.theta );

					// todo : sensor pose configuration
					// not moving in wheel odometry data
					if( msg_vel2d.vel_x == 0 && msg_vel2d.vel_y == 0 && msg_vel2d.vel_ang == 0 &&
							fabs(msg_imu.Gyro.z - offset_rate) < gnd_deg2ang(2.0) ) {
						msg_imu_t ws;

						if( time_current - time_vehiclestop > node_config.offset_calibration_time_margin.value * 2.0
								&& msgreader_imu.copy_at_time(&ws, time_current - node_config.offset_calibration_time_margin.value) == 0 ) {
							// update offset
							offset_rate += ( (-ws.Gyro.z) - offset_rate) * node_config.offset_calibration_factor.value;
							rate = 0;
						}
					}
					// moving
					else {
						time_vehiclestop = time_current;
						rate = (-msg_imu.Gyro.z) - offset_rate;
					}

					// calculate robot direction
					msg_gyrodo.header.seq++;
					msg_gyrodo.header.stamp = msg_vel2d.header.stamp;
					msg_gyrodo.x += (msg_vel2d.vel_x * cosv - msg_vel2d.vel_y * sinv) * node_config.cycle.value;
					msg_gyrodo.y += (msg_vel2d.vel_x * sinv + msg_vel2d.vel_y * cosv) * node_config.cycle.value;
					msg_gyrodo.theta += rate * node_config.cycle.value;

					// publish
					pub_gyrodo.publish(msg_gyrodo);

					if( fp_textlog ) {
						fprintf(fp_textlog, "%lf %lf %lf %lf %lf\n", time_current - time_start, msg_gyrodo.x, msg_gyrodo.y, msg_gyrodo.theta, offset_rate );
					}

					// update period
					time_gyrodom = gnd_loop_next(time_current, time_start, node_config.cycle.value);
				} // <--- pose calculation
			}
			// <--- calculate 2d pose

			// ---> show status
			if( node_config.status_display.value && time_current > time_display ) {
				msg_imu_t ws;

				if( nline_display ) {
					::fprintf(stderr, "\x1b[%02dA", nline_display);
					nline_display = 0;
				}

				nline_display++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_display++; ::fprintf(stderr, "\x1b[K operating time : %6.01lf[sec]\n", time_current - time_start);
				// position (publish)
				nline_display++; ::fprintf(stderr, "\x1b[K      position  :   topic name \"%s\" (publish)\n", node_config.topic_name_gyrodom.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :        value %8.03lf[m], %8.03lf[m], %6.01lf[deg]\n",
						msg_gyrodo.x, msg_gyrodo.y, gnd_ang2deg( msg_gyrodo.theta ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                :     pub rate %lf\n",
						(double) (msg_gyrodo.header.seq - seq_gyrodo_prevdisp) / cycle_display);
				seq_gyrodo_prevdisp = msg_gyrodo.header.seq;
				// velocity (subscribe)
				nline_display++; ::fprintf(stderr, "\x1b[K  velocity (2D) :   topic name \"%s\" (subscribe)\n", node_config.topic_name_vel2d.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :        value %5.02lf[m/s], %4.02lf[m/s], %4.01lf[deg/s]\n",
						msg_vel2d.vel_x, msg_vel2d.vel_y, gnd_ang2deg( msg_vel2d.vel_ang ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                :   subsc rate %lf\n",
						(double) (msg_vel2d.header.seq - seq_vel2d_prevdisp) / cycle_display);
				seq_vel2d_prevdisp = msg_vel2d.header.seq;
				// imu (subscribe)
				msgreader_imu.copy_latest(&ws);
				nline_display++; ::fprintf(stderr, "\x1b[K    gyro sensor :   topic name \"%s\" (subscribe)\n", node_config.topic_name_imu.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :     yaw rate %4.01lf[deg/s], offset corrected %4.01lf[deg/s]\n", gnd_ang2deg( -ws.Gyro.z ),  gnd_ang2deg( -ws.Gyro.z - offset_rate ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                :   subsc rate %lf\n",
						(double) (ws.header.seq - seq_imu_prevdisp) / cycle_display);
				seq_imu_prevdisp = ws.header.seq;
				nline_display++; ::fprintf(stderr, "\x1b[K                :  rate offset %6.04lf[deg/s]\n", gnd_ang2deg( offset_rate ) );

				nline_display++; ::fprintf(stderr, "\x1b[K\n");

				// update
				time_display = gnd_loop_next(time_current, time_start, cycle_display);

			} // <--- show status

		} // <--- main loop

	} // <--- operation



	{ // ---> finalize
		fprintf(stdout, "---------- finalize ----------\n");
		if( fp_textlog ) fclose( fp_textlog );
	} // <--- finalize
	fprintf(stdout, "  ... fin\n");

	return 0;
}
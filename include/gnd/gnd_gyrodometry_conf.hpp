/*
 * gnd_gyrodometry_conf.hpp
 *
 *  Created on: 2014/08/01
 *      Author: tyamada
 */

#ifndef GND_GYRODOMETRY_CONF_HPP_
#define GND_GYRODOMETRY_CONF_HPP_

#include <string.h>

#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-util.h"
#include "gnd/gnd-lib-error.h"

// ---> type declaration
namespace gnd {
	namespace gyrodometry {
		struct node_config;
		typedef struct node_config node_config;

		typedef gnd::conf::parameter_array<char, 256> param_string_t;
		typedef gnd::conf::param_int param_int_t;
		typedef gnd::conf::param_double param_double_t;
		typedef gnd::conf::param_bool param_bool_t;
	}
} // <--- type declaration


// ---> const variables definition
namespace gnd {
	namespace gyrodometry {
		static const param_string_t Default_node_name = {
				"node-name",
				"gnd_gyrodometry_node",
				"ros-node name"
		};

		static const param_string_t Default_topic_name_imu = {
				"topic-name-imu",
				"imu",
				"imu topic name (subscribe)"
		};

		static const param_string_t Default_topic_name_vel2d = {
				"topic-name-velocity",
				"vehiclevel",
				"2d velocity topic name (subscribe)"
		};

		static const param_string_t Default_topic_name_gyrodometry = {
				"topic-name-gyrodometry",
				"gyrodometry",
				"gyrodometry(pose2d) topic name (publish)"
		};

		static const param_double_t Default_cycle = {
				"cycle",
				gnd_msec2sec(5),
				"position estimation cycle [sec]"
		};

		static const param_double_t Default_offset_calibration_default = {
				"offset-calibration-default",
				0.0,
				"offset calibration default"
		};

		static const param_double_t Default_offset_calibration_factor = {
				"offset-calibration-factor",
				0.005,
				"offset calibration factor"
		};

		static const param_double_t Default_offset_calibration_time_margin = {
				"offset-calibration-time-margin",
				0.1,
				"offset calibration start after this time from robot stopped"
		};

		static const param_double_t Default_cycle_cui_status_display = {
				"cycle-cui-status-display",
				0,
				"cycle to display the node status in standard output [msec]. [note] it need ansi color code. [note] if this value is less than or equal 0, don't display"
		};
		// ---> debug condition


		static const param_string_t Default_gyrodometry_log = {
				"gyrodometry-log",
				"",
				"gyrodometry log(text file)"
		};

	}
}
// <--- const variables definition



// ---> function declaration
namespace gnd {
	namespace gyrodometry {
		/**
		 * @brief initialize configure to default parameter
		 * @param [out] p : node_config
		 */
		int init_node_config(node_config *conf);


		/**
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		int fread_node_config( const char* fname, node_config *dest );
		/**
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		int get_node_config( node_config *dest, gnd::conf::configuration *src );



		/**
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		int fwrite_node_config( const char* fname, node_config *src );
		/**
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		int set_node_config( gnd::conf::configuration *dest, node_config *src );


	}
} // <--- function declaration



// ---> type definition
namespace gnd {
	namespace gyrodometry {
		/**
		 * \brief configuration parameter for gnd_gyrodometry node
		 */
		struct node_config {
			node_config();

			// ros communication config
			param_string_t	node_name;							///< node name
			param_string_t	topic_name_imu;						///< imu topic name
			param_string_t	topic_name_vel2d;					///< vel2d topic name
			param_string_t	topic_name_gyrodom;					///< gyrodometory topic name

			// position estimation config
			param_double_t	cycle;								///< position estimation cycle
			param_double_t	offset_calibration_default;			///< offset calibration option
			param_double_t	offset_calibration_time_margin;		///< offset calibration option
			param_double_t	offset_calibration_factor;			///< offset calibration option

			// debug option
			param_string_t	gyrodometry_log;					///< gyrodometory topic name
			param_double_t	cycle_cui_status_display;			///< cui status display mode
		};

		inline
		node_config::node_config() {
			init_node_config(this);
		}
		// <--- struct node_config
	}
}
// <--- type definition



// ---> function definition
namespace gnd {
	namespace gyrodometry {
		/*
		 * \brief initialize configuration parameter
		 * @param [out] p : node_config
		 */
		inline
		int init_node_config( node_config *p ){
			gnd_assert(!p, -1, "invalid null pointer argument\n" );

			// ros communication
			memcpy( &p->node_name,						&Default_node_name,							sizeof(Default_node_name) );
			memcpy( &p->topic_name_imu,					&Default_topic_name_imu,					sizeof(Default_topic_name_imu) );
			memcpy( &p->topic_name_vel2d,				&Default_topic_name_vel2d,					sizeof(Default_topic_name_vel2d) );
			memcpy( &p->topic_name_gyrodom,				&Default_topic_name_gyrodometry,			sizeof(Default_topic_name_gyrodometry) );
			// position estimation
			memcpy( &p->cycle,							&Default_cycle,								sizeof(Default_cycle) );
			memcpy( &p->offset_calibration_default,		&Default_offset_calibration_default,		sizeof(Default_offset_calibration_default) );
			memcpy( &p->offset_calibration_factor,		&Default_offset_calibration_factor,			sizeof(Default_offset_calibration_factor) );
			memcpy( &p->offset_calibration_time_margin,	&Default_offset_calibration_time_margin,	sizeof(Default_offset_calibration_time_margin) );
			// debug option
			memcpy( &p->gyrodometry_log,				&Default_gyrodometry_log,					sizeof(Default_gyrodometry_log) );
			memcpy( &p->cycle_cui_status_display,		&Default_cycle_cui_status_display,			sizeof(Default_cycle_cui_status_display) );

			return 0;
		}

		/*
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int fread_node_config( const char* fname, node_config *dest ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(fname)) < 0 )    return ret;

				return get_node_config(dest, &fs);
			} // <--- operation
		}
		/*
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		inline
		int get_node_config( node_config *dest, gnd::conf::configuration *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication
			gnd::conf::get_parameter( src, &dest->node_name );
			gnd::conf::get_parameter( src, &dest->topic_name_imu );
			gnd::conf::get_parameter( src, &dest->topic_name_vel2d );
			gnd::conf::get_parameter( src, &dest->topic_name_gyrodom );
			// position estimation
			gnd::conf::get_parameter( src, &dest->cycle );
			gnd::conf::get_parameter( src, &dest->offset_calibration_default );
			gnd::conf::get_parameter( src, &dest->offset_calibration_factor );
			gnd::conf::get_parameter( src, &dest->offset_calibration_time_margin );
			// debug option
			gnd::conf::get_parameter( src, &dest->gyrodometry_log );
			gnd::conf::get_parameter( src, &dest->cycle_cui_status_display );
			return 0;
		}



		/*
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		inline
		int fwrite_node_config( const char* fname, node_config *src ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );
			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

				return fs.write(fname);
			} // <--- operation
		}

		/*
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		inline
		int set_node_config( gnd::conf::configuration *dest, node_config *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication
			gnd::conf::set_parameter(dest, &src->node_name );
			gnd::conf::set_parameter(dest, &src->topic_name_imu );
			gnd::conf::set_parameter(dest, &src->topic_name_vel2d );
			gnd::conf::set_parameter(dest, &src->topic_name_gyrodom );
			// position estimation
			gnd::conf::set_parameter(dest, &src->cycle );
			gnd::conf::set_parameter(dest, &src->offset_calibration_default );
			gnd::conf::set_parameter(dest, &src->offset_calibration_factor );
			gnd::conf::set_parameter(dest, &src->offset_calibration_time_margin );
			// debug option
			gnd::conf::set_parameter(dest, &src->gyrodometry_log );
			gnd::conf::set_parameter(dest, &src->cycle_cui_status_display );

			return 0;
		}

	}
}
// <--- function definition




#endif /* GND_GYRODOMETRY_CONF_HPP_ */

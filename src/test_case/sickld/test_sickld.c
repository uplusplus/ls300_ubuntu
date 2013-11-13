/*!
 * \file test_sickld.c
 * \brief sickld test
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */
#include "../hd_test_config.h"
#if TEST_SICKLD

#if 1

#include <sickld/sickld.h>
/**
 * \brief Computes the active scan area for the Sick given the current
 *        sector configuration
 * \param sick_angle_step The angular resolution of the Sick LD
 * \param active_sector_start_angles The start angles for the active scan sectors
 * \param active_sector_stop_angles The stop angles for the active scan sectors
 * \param num_active_sectors The number of active sectors
 * \return The Sick LD scan area corresponding to the given bounds
 *
 * NOTE: The Sick LD computes scan area by subtracting the end of the
 *       previous sector from the end of the current sector.  As such, we have
 *       to add a single step angle to our computation in order to get the scan
 *       area that is used by the Sick LD.  Unfortunately, this is how the device
 *       does its computation (as opposed to using the angle at which the first
 *       scan in the given sector is taken) so this is how we do it.
 *
 */
static e_float64 sld_compute_scan_area(const e_float64 sick_angle_step,
		const e_float64 * const active_sector_start_angles,
		const e_float64 * const active_sector_stop_angles,
		const e_uint32 num_active_sectors)
{

	/* Define the current scan area */
	e_float64 total_scan_area = 0;
	e_float64 curr_sector_scan_area = 0;
	e_uint32 i;

	/* For each sector given sum the absolute scan area for it */
	for (i = 0; i < num_active_sectors; i++)
			{

		/* Compute the total scan area for this sector */
		curr_sector_scan_area = fabs(
										active_sector_start_angles[i]
												- active_sector_stop_angles[i]);

		/* Update the total scan area */
		total_scan_area += curr_sector_scan_area + sick_angle_step;
	}

	/* Return the computed area */
	return total_scan_area;

}

/**
 * \brief Compute the mean pulse frequency (see page 22 of the operator's manual)
 * \param active_scan_area The total scan area that can be covered by the Sick
 * \param curr_motor_speed The current motor speed (in Hz)
 * \param curr_angular_resolution The current angular resolution of the Sick (in deg)
 * \return The maximum pulse frequency for the given configuration parameters
 */
static e_float64 sld_compute_max_pulse_frequency(
		const e_float64 total_scan_area, const e_float64 curr_motor_speed,
		const e_float64 curr_angular_resolution)
{
	/* Compute the maximum pulse frequency */
	return total_scan_area * curr_motor_speed * (1 / curr_angular_resolution);
}

/**
 * \brief Compute the mean pulse frequency (see page 22 of the operator's manual)
 * \param active_scan_area The total area where the Sick is actively scanning (in deg) (i.e. total area where the laser isn't blanked)
 * \param curr_motor_speed The current motor speed (in Hz)
 * \param curr_angular_resolution The current angular resolution of the Sick (in deg)
 * \return The mean pulse frequency for the given configuration parameters
 */
static e_float64 sld_compute_mean_pulse_frequency(
		const e_float64 active_scan_area, const e_float64 curr_motor_speed,
		const e_float64 curr_angular_resolution)
{
	/* Compute the mean pulse frequency */
	return sld_compute_max_pulse_frequency(SICK_MAX_SCAN_AREA, curr_motor_speed,
											curr_angular_resolution)
			* (active_scan_area / ((e_float64) SICK_MAX_SCAN_AREA));
}



static e_int32 sld_valid_pulse_frequency_ex(const e_uint32 sick_motor_speed,
		const e_float64 sick_angle_step,
		const e_float64 * const active_sector_start_angles,
		const e_float64 * const active_sector_stop_angles,
		const e_uint32 num_active_sectors)
{

	/* Compute the scan area */
	e_float64 scan_area =
			sld_compute_scan_area(sick_angle_step,
									active_sector_start_angles, active_sector_stop_angles,
									num_active_sectors);

	/* Check the mean pulse rate of the desired configuration */
	if ( sld_compute_mean_pulse_frequency(scan_area, sick_motor_speed,
											sick_angle_step) > SICK_MAX_MEAN_PULSE_FREQUENCY )
											{
		DMSG((STDOUT,"Max mean pulse frequency exceeded! (try a slower motor speed,"
		"a larger step angle and/or a smaller active scan area)\r\n"));
		return E_ERROR;
	}

	/* Check the maximum pulse rate of the desired configuration */
	if ( sld_compute_max_pulse_frequency(SICK_MAX_SCAN_AREA, sick_motor_speed,
											sick_angle_step) > SICK_MAX_PULSE_FREQUENCY )
	{
		DMSG((STDOUT, "Max pulse frequency exceeded! (try a slower motor speed,"
		"a larger step angle and/or a smaller active scan area)\r\n"));
		return E_ERROR;
	}

	/* Valid! */
	return E_OK;
}
int main()
{

	e_float64 sec[] = { 0,270-0.25 };

	int i = sld_valid_pulse_frequency_ex(10,0.25,sec,sec+1,1);

	printf("%d",i);

	return 0;
}

#else

#include <stdlib.h>
#include <sickld/sickld.h>

int main(int argc, char *argv[])
{
	e_int32 ret, i;
	/* A string for the IP address */
	char* sick_ip_addr = DEFAULT_SICK_IP_ADDRESS;

	/* Check the num of args */
	if (argc > 2 || (argc == 2 && strcasecmp(argv[1], "--help") == 0))
	{
		DMSG((STDOUT,"Usage: ld_single_sector [SICK IP ADDRESS]\r\n"
		"Ex. ld_single_sector 192.168.1.11\r\n"));
		return -1;
	}

	/* Assign the IP address */
	if (argc == 2)
	{
		sick_ip_addr = argv[1];
	}

	/* Define the data buffers */
	/* Define the data buffers */
	double values[SICK_MAX_NUM_MEASUREMENTS] =
	{ 0 };
	e_uint32 num_values = 0;

	/* Define the bounds for a single sector */
	double sector_start_ang = 100;
	double sector_stop_ang = 270;

	/* Define the object */
	sickld_t* sickld;
	ret = sld_create(&sickld, sick_ip_addr, DEFAULT_SICK_TCP_PORT);
	if (e_failed(ret))
		goto OUT1;

	/*
	 * Initialize the Sick LD
	 */
	ret = sld_initialize(sickld);
	if (e_failed(ret))
		goto OUT2;

	DMSG((STDOUT, "\tInitialize end.\r\n"));

	/* Set the desired sector configuration */
	ret = sld_set_temp_scan_areas(sickld, &sector_start_ang, &sector_stop_ang,
			1);
	if (e_failed(ret))
		goto OUT2;

	/* Print the sector configuration */
	ret = sld_print_sector_config(sickld);
	if (e_failed(ret))
		goto OUT2;

	/* Acquire some range measurements */
	for (i = 0; i < 10; i++)
	{
		/* Here we only want the range values so the second arg is NULL */
		ret = sld_get_measurements(sickld, values, NULL, NULL, &num_values,
				NULL, NULL, NULL, NULL, NULL, NULL, NULL);
		if (e_failed(ret))
			goto OUT2;
		DMSG((STDOUT,"\t Num. Valuse: %u\r\n",num_values));
	}

	OUT2: sld_uninitialize(sickld);
	OUT1: sld_release(&sickld);
	return ret;
}
#endif

#endif /*SICKLD*/

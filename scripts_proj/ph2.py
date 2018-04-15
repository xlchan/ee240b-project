import pprint

import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as interp
import scipy.optimize as opt
import scipy.signal as sig

from bag.util.search import BinaryIterator
from bag.util.search import minimize_cost_golden_float
from bag.data.lti import LTICircuit
from bag.data.lti import get_w_3db
from bag.data.lti import get_stability_margins
from verification_ec.mos.query import MOSDBDiscrete

# Additional constraints
# Channel length of 45 nm:
# pmos_spec = 'specs_mos_char/pch_w0d5.yaml'
# nmos_spec = 'specs_mos_char/nch_w0d5.yaml'
# LVT threshold flavor:
# intent = 'lvt'

def get_db(spec_file, intent, interp_method='spline', sim_env='tt'):
	# Initialize transistor database from simulation data
	mos_db = MOSDBDiscrete([spec_file], interp_method=interp_method)
	# Set process corners
	mos_db.env_list = [sim_env]
	# Set layout parameters
	mos_db.set_dsn_params(intent=intent)
	return mos_db

def design_CML(specs):
	sim_env = specs['sim_env']
	db = specs['db']
	vgs_res = specs['vgs_res']
	vds_res = specs['vds_res']
	vdd = specs['vdd']
	t_rise = specs['t_rise']
	vin_min = specs['vin_min']
	vid_min = vin_min * 2
	t_per = specs['t_per']
	# t_factor = specs['t_factor']
	# t_bw = t_factor * t_per
	c_yield = specs['c_yield']
	vin_off = specs['vin_off']
	cff = specs['cff']
	t_setup = specs['t_setup']
	vout_min = specs['vout_min']
	scale_max = specs['scale_max']

	# SS parameter functions
	ibias_fun = db.get_function('ibias', env=sim_env)
	gm_fun = db.get_function('gm', env=sim_env)
	gds_fun = db.get_function('gds', env=sim_env)
	cgg_fun = db.get_function('cgg', env=sim_env)
	cdd_fun = db.get_function('cdd', env=sim_env)

	# Primary timing constraint
	t_clk = t_per - 2 * (t_setup + t_rise)

	# get Vgs/Vds sweep values
	vgs_idx = db.get_fun_arg_index('vgs')
	vds_idx = db.get_fun_arg_index('vds')
	vgs_min, vgs_max = ibias_fun.get_input_range(vgs_idx)
	vds_min, vds_max = ibias_fun.get_input_range(vds_idx)
	vds_max = min(vds_max, vdd - vds_res)
	num_vgs = int(np.ceil((vgs_max - vgs_min) / vgs_res)) + 1
	num_vds = int(np.ceil((vds_max - vds_min) / vds_res)) + 1
	vgs_vec = np.linspace(vgs_min, vgs_max, num_vgs, endpoint=True)
	vds_vec = np.linspace(vds_min, vds_max, num_vds, endpoint=True)

	# Sweep Vgs and Vds to find best operating point such that Vout settles fast enough
	for vgs in vgs_vec:
		for vds in vds_vec:
			# Setup bias
			arg_in = db.get_fun_arg(vgs=vgs, vds=vds, vbs=0)
			ibias_in = ibias_fun(arg_in)
			arg_xc = db.get_fun_arg(vgs=vds, vds=vds, vbs=0)
			ibias_xc = ibias_fun(arg_xc)

			# Check that VDS is not too small
			if ibias_xc < 0 or ibias_in < 0:
				continue

			# Extract small signal parameters
			if ibias_xc > ibias_in:
				rload = (vdd - vds) / ibias_xc
				if 2 * ibias_xc * rload < vout_min:
					continue
				scale = ibias_xc / ibias_in
				gm_in = gm_fun(arg_in) * scale
				gds_in = gds_fun(arg_in) * scale
				cdd_in = cdd_fun(arg_in) * scale
				gm_xc = gm_fun(arg_xc)
				gds_xc = gds_fun(arg_xc)
				cgg_xc = cgg_fun(arg_xc)
				cdd_xc = cdd_fun(arg_xc)
			else:
				rload = (vdd - vds) / ibias_in
				if 2 * ibias_in * rload < vout_min:
					continue
				scale = ibias_in / ibias_xc
				gm_in = gm_fun(arg_in)
				gds_in = gds_fun(arg_in)
				cdd_in = cdd_fun(arg_in)
				gm_xc = gm_fun(arg_xc) * scale
				gds_xc = gds_fun(arg_xc) * scale
				cgg_xc = cgg_fun(arg_xc) * scale
				cdd_xc = cdd_fun(arg_xc) * scale

			if scale > scale_max:
				continue

			# Consolidate design equations
			go = gds_in + 1 / rload
			co = cff + cdd_xc + cdd_in + cgg_xc
			bw_in = go / co
			av_in = gm_in / go
			bw_xc = (gds_xc - gm_xc + 1 / rload) / co
			# bw_xc = (-gm_xc + 1 / rload) / co
			vo_in = ibias_in * 2 * rload * np.exp((-t_clk / 2 + t_setup) * bw_in) + av_in * vid_min * (1 - np.exp((-t_clk / 2 + t_setup) * bw_in))
			vo_xc = vo_in * np.exp(t_clk / 2 * bw_xc)

			# Check output has setled to correct value in given time
			if vo_xc >= vout_min and vo_xc < vdd:
				CML_design = dict(
					ibias_in_uA=ibias_in * 1e6,
					ibias_xc_uA=ibias_xc * 1e6,
					scale=scale,
					vout_mV=vo_xc * 1e3,
					rload_kOhms=rload * 1e-3,
					vgs=vgs,
					vds=vds
					)
				return CML_design




def run_main():
	nmos_spec = 'specs_mos_char/nch_w0d5.yaml'
	intent = 'lvt'
	interp_method = 'linear'
	sim_env = 'tt'

	db = get_db(nmos_spec, intent, interp_method=interp_method, sim_env=sim_env)

	specs = dict(
		sim_env=sim_env,
		db=db,
		vgs_res=5e-3,
		vds_res=5e-3,
		vdd=1.2,
		t_rise=20e-12,
		vin_min=20e-3,
		t_per=200e-12,
		# t_factor=3/2,
		c_yield=0,
		vin_off=float('inf'),
		cff=5e-15,
		t_setup=30e-12,
		vout_min=200e-3, #800e-3
		scale_min=1,
		scale_max=10,
		n_scale=100,
		)

	CML = design_CML(specs)
	pprint.pprint(CML)
		

if __name__ == '__main__':
	run_main()
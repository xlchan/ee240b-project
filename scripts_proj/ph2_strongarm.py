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

def design_StrongArm(specs, input_op, xcoupled_op):

def design_xcoupled(specs, input_op):

def design_input(specs):
	sim_env = specs['sim_env']
	db = specs['nch_db']
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
	t_high = t_per / 2 - t_setup - t_rise

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


def run_main():
	nmos_spec = 'specs_mos_char/nch_w0d5.yaml'
	pmos_spec = 'specs_mos_char/pch_w0d5.yaml'
	intent = 'lvt'
	interp_method = 'linear'
	sim_env = 'tt'

	nch_db = get_db(nmos_spec, intent, interp_method=interp_method, sim_env=sim_env)
	pch_db = get_db(pmos_spec, intent, interp_method=interp_method, sim_env=sim_env)

	specs = dict(
		sim_env=sim_env,
		nch_db=nch_db,
		pch_db=pch_db,
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
		vout_min=800e-3, #200e-3
		scale_min=1,
		scale_max=10,
		n_scale=100,
		)
		

if __name__ == '__main__':
	run_main()
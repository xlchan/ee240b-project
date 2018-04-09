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

def design_input(specs, load_op_info):
	sim_env = specs['sim_env']
	nch_db = specs['nch_db']
	vgs_res = specs['vgs_res']
	vdd = specs['vdd']
	t_rise = specs['t_rise']
	vin_min = specs['vin_min']
	vid_min = vin_min * 2
	t_per = specs['t_per']
	t_factor = specs['t_factor']
	t_bw = t_factor * t_per
	c_yield = specs['c_yield']
	vin_off = specs['vin_off']
	cff = specs['cff']
	t_setup = specs['t_setup']
	vout_min = specs['vout_mi']

	load_gm = load_op_info['gm']
	load_cp = load_op_info['cp']
	w_regen = load_gm / load_cp

	t_high = t_bw / 2 - t_setup - 2 * t_rise

	vgs_idx = nch_db.get_fun_arg_index('vgs')
    ibias_fun = nch_db.get_function('ibias', env=sim_env)
    vgs_min, vgs_max = ibias_fun.get_input_range(vgs_idx)

    def zero_fun1(t_int, vgs):
    	arg = nch_db.get_fun_arg(vgs=vgs, vds=vdd, vbs=0)
    	ibias

    num_pts = int(np.ceil((vgs_max - vgs_min) / vgs_res)) + 1
    for vgs_val in np.linspace(vgs_min, vgs_max, num_pts, endpoint=True):
		input_op_info = nch_db.query(vgs=vgs_val, vds=vdd, vbs=0)
		icm = input_op_info['ibias']
		gm = input_op_info['gm']
		cp = load_op_info['cgs_n'] + load_op_info['cds_n']




def design_load(specs):
	# May need this spec for future designs
	c_yield = specs['c_yield']
	vin_off = specs['vin_off']
	pch_db = specs['pch_db']
	nch_db = specs['nch_db']

	# Set all transistors to be minimally sized
	pch_op_info = pch_db.query()
	nch_op_info = nch_db.query()

	return 

def run_main():
    pmos_spec = 'specs_mos_char/pch_w0d5.yaml'
    nmos_spec = 'specs_mos_char/nch_w0d5.yaml'
    intent = 'lvt'
    interp_method = 'linear'
    sim_env = 'tt'

    pch_db = get_db(pmos_spec, intent, interp_method=interp_method, sim_env=sim_env)
    nch_db = get_db(nmos_spec, intent, interp_method=interp_method, sim_env=sim_env)

    specs = dict(
        sim_env=sim_env,
        pch_db=pch_db,
        nch_db=nch_db,
        vgs_res=5e-3,
        vdd=1.2,
        t_rise=20e-12,
        vin_min=20e-3,
        t_per=200e-12,
        t_factor=3/2,
        c_yield=0,
        vin_off=float('inf'),
        cff=5e-15,
        t_setup=30e-12,
        vout_min=200e-3, #800e-3
        # scale_min=1, # 1
        # scale_max=10, # 2.5
        # n_scale=100,
        )

    pch_load_info = design_load(specs)
    input_specs = design_input(specs, pch_load_info)
    pprint.pprint(input_specs)
    	

if __name__ == '__main__':
    run_main()
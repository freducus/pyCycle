import sys
import numpy as np

import openmdao.api as om

import pycycle.api as pyc

class MultiSpoolTurboshaft(pyc.Cycle):

    def initialize(self):
        self.options.declare('maxiter', default=20,
                              desc='Maximum number of Newton solver iterations.')
        super().initialize()

    def setup(self):

        design = self.options['design']
        maxiter = self.options['maxiter']
        self.options['thermo_method'] = 'CEA'
        self.options['thermo_data'] = pyc.species_data.janaf

        self.add_subsystem('fc', pyc.FlightConditions())
        self.add_subsystem('inlet', pyc.Inlet())
        self.add_subsystem('duct1', pyc.Duct())
        self.add_subsystem('lpc', pyc.Compressor(map_data=pyc.LPCMap),
                           promotes_inputs=[('Nmech','IP_Nmech')])
        self.add_subsystem('icduct', pyc.Duct())
        self.add_subsystem('hpc_axi', pyc.Compressor(map_data=pyc.HPCMap),
                           promotes_inputs=[('Nmech','HP_Nmech')])
        self.add_subsystem('bld25', pyc.BleedOut(bleed_names=['cool1','cool2']))
        # self.add_subsystem('hpc_centri', pyc.Compressor(map_data=pyc.HPCMap),
        #                    promotes_inputs=[('Nmech','HP_Nmech')])
        # self.add_subsystem('bld3', pyc.BleedOut(bleed_names=['cool3','cool4']))
        self.add_subsystem('duct6', pyc.Duct())
        self.add_subsystem('burner', pyc.Combustor(fuel_type='Jet-A(g)'))
        self.add_subsystem('hpt', pyc.Turbine_ITP(map_data=pyc.HPTMap, bleed_names=['cool1']),
                           promotes_inputs=[('Nmech','HP_Nmech')])
        self.add_subsystem('duct43', pyc.Duct())
        self.add_subsystem('lpt', pyc.Turbine_ITP(map_data=pyc.LPTMap, bleed_names=['cool2']),
                           promotes_inputs=[('Nmech','IP_Nmech')])
        self.add_subsystem('itduct', pyc.Duct())
        self.add_subsystem('pt', pyc.Turbine(map_data=pyc.LPTMap),
                           promotes_inputs=[('Nmech','LP_Nmech')])
        self.add_subsystem('duct12', pyc.Duct())
        self.add_subsystem('nozzle', pyc.Nozzle(nozzType='CV', lossCoef='Cv'))
        #
        self.add_subsystem('lp_shaft', pyc.Shaft(num_ports=1),promotes_inputs=[('Nmech','LP_Nmech')])
        self.add_subsystem('ip_shaft', pyc.Shaft(num_ports=2),promotes_inputs=[('Nmech','IP_Nmech')])
        self.add_subsystem('hp_shaft', pyc.Shaft(num_ports=2),promotes_inputs=[('Nmech','HP_Nmech')])
        self.add_subsystem('perf', pyc.Performance(num_nozzles=1, num_burners=1))

        self.connect('duct1.Fl_O:tot:P', 'perf.Pt2')
        self.connect('hpc_axi.Fl_O:tot:P', 'perf.Pt3')
        self.connect('burner.Wfuel', 'perf.Wfuel_0')
        self.connect('inlet.F_ram', 'perf.ram_drag')
        self.connect('nozzle.Fg', 'perf.Fg_0')
        self.connect('lp_shaft.pwr_in', 'perf.power')
        #
        self.connect('pt.trq', 'lp_shaft.trq_0')
        self.connect('lpc.trq', 'ip_shaft.trq_0')
        self.connect('lpt.trq', 'ip_shaft.trq_1')
        self.connect('hpc_axi.trq', 'hp_shaft.trq_0')
        # self.connect('hpc_centri.trq', 'hp_shaft.trq_1')
        self.connect('hpt.trq', 'hp_shaft.trq_1')
        self.connect('fc.Fl_O:stat:P', 'nozzle.Ps_exhaust')

        balance = self.add_subsystem('balance', om.BalanceComp())
        if design:

            # balance.add_balance('W', units='kg/s', eq_units=None)
            # self.connect('balance.W', 'fc.W')
            # self.connect('nozzle.PR', 'balance.lhs:W')

            balance.add_balance('FAR', eq_units='degK', lower=1e-4, val=.017, rhs_name='T4_target')
            self.connect('balance.FAR', 'burner.Fl_I:FAR')
            # self.connect('burner.Fl_O:tot:T', 'balance.lhs:FAR')
            self.connect('hpt.st_45:tot:T', 'balance.lhs:FAR')

            balance.add_balance('lpt_PR', val=1.5, lower=1.001, upper=8, eq_units='kW', rhs_val=0.)
            self.connect('balance.lpt_PR', 'lpt.PR')
            self.connect('ip_shaft.pwr_net', 'balance.lhs:lpt_PR')

            balance.add_balance('hpt_PR', val=1.5, lower=1.001, upper=8, eq_units='kW', rhs_val=0.)
            self.connect('balance.hpt_PR', 'hpt.PR')
            self.connect('hp_shaft.pwr_net', 'balance.lhs:hpt_PR')

            balance.add_balance('pt_PR', val=1.5, lower=1.001, upper=15, eq_units='kW',  rhs_name='pwr_target')
            self.connect('balance.pt_PR', 'pt.PR')
            self.connect('lp_shaft.pwr_net', 'balance.lhs:pt_PR')


        # else:
        #     balance.add_balance('FAR', eq_units='hp', lower=1e-4, val=.017)
        #     self.connect('balance.FAR', 'burner.Fl_I:FAR')
        #     self.connect('lp_shaft.pwr_net', 'balance.lhs:FAR')
        #
        #     balance.add_balance('W', units='lbm/s', eq_units='inch**2')
        #     self.connect('balance.W', 'inlet.Fl_I:stat:W')
        #     self.connect('nozzle.Throat:stat:area', 'balance.lhs:W')
        #
        #     balance.add_balance('IP_Nmech', val=12000.0, units='rpm', lower=1.001, eq_units='hp', rhs_val=0.)
        #     self.connect('balance.IP_Nmech', 'IP_Nmech')
        #     self.connect('ip_shaft.pwr_net', 'balance.lhs:IP_Nmech')
        #
        #     balance.add_balance('HP_Nmech', val=14800.0, units='rpm', lower=1.001, eq_units='hp', rhs_val=0.)
        #     self.connect('balance.HP_Nmech', 'HP_Nmech')
        #     self.connect('hp_shaft.pwr_net', 'balance.lhs:HP_Nmech')

        self.pyc_connect_flow('fc.Fl_O', 'inlet.Fl_I', connect_w=True)
        self.pyc_connect_flow('inlet.Fl_O', 'duct1.Fl_I')
        self.pyc_connect_flow('duct1.Fl_O', 'lpc.Fl_I')
        self.pyc_connect_flow('lpc.Fl_O', 'icduct.Fl_I')
        self.pyc_connect_flow('icduct.Fl_O', 'hpc_axi.Fl_I')
        self.pyc_connect_flow('hpc_axi.Fl_O', 'bld25.Fl_I')
        # self.pyc_connect_flow('bld25.Fl_O', 'hpc_centri.Fl_I')

        self.pyc_connect_flow('bld25.Fl_O', 'duct6.Fl_I')

        # self.pyc_connect_flow('hpc_centri.Fl_O', 'bld3.Fl_I')
        # self.pyc_connect_flow('bld3.Fl_O', 'duct6.Fl_I')
        self.pyc_connect_flow('duct6.Fl_O', 'burner.Fl_I')
        self.pyc_connect_flow('burner.Fl_O', 'hpt.Fl_I')
        self.pyc_connect_flow('hpt.Fl_O', 'duct43.Fl_I')
        self.pyc_connect_flow('duct43.Fl_O', 'lpt.Fl_I')
        self.pyc_connect_flow('lpt.Fl_O', 'itduct.Fl_I')
        self.pyc_connect_flow('itduct.Fl_O', 'pt.Fl_I')
        self.pyc_connect_flow('pt.Fl_O', 'duct12.Fl_I')
        self.pyc_connect_flow('duct12.Fl_O', 'nozzle.Fl_I')
        #
        self.pyc_connect_flow('bld25.cool1', 'hpt.cool1', connect_stat=False)
        self.pyc_connect_flow('bld25.cool2', 'lpt.cool2', connect_stat=False)
        # self.pyc_connect_flow('bld3.cool3', 'hpt.cool3', connect_stat=False)
        # self.pyc_connect_flow('bld3.cool4', 'hpt.cool4', connect_stat=False)

        newton = self.nonlinear_solver = om.NewtonSolver()
        newton.options['atol'] = 1e-6
        newton.options['rtol'] = 1e-6
        newton.options['iprint'] = 2
        newton.options['maxiter'] = maxiter
        newton.options['solve_subsystems'] = True
        newton.options['max_sub_solves'] = 100
        newton.options['reraise_child_analysiserror'] = False
        newton.linesearch = om.BoundsEnforceLS()
        newton.linesearch.options['bound_enforcement'] = 'scalar'
        newton.linesearch.options['iprint'] = -1

        self.linear_solver = om.DirectSolver()

        super().setup()

def viewer(prob, pt, file=sys.stdout):
    """
    print a report of all the relevant cycle properties
    """

    summary_data = (prob[pt+'.fc.Fl_O:stat:MN'], prob[pt+'.fc.alt'],prob[pt+'.inlet.Fl_O:stat:W'], 
                    prob[pt+'.perf.Fn'], prob[pt+'.perf.Fg'], prob[pt+'.inlet.F_ram'],
                    prob[pt+'.perf.OPR'], prob[pt+'.perf.PSFC'])

    print(file=file, flush=True)
    print(file=file, flush=True)
    print(file=file, flush=True)
    print("----------------------------------------------------------------------------", file=file, flush=True)
    print("                              POINT:", pt, file=file, flush=True)
    print("----------------------------------------------------------------------------", file=file, flush=True)
    print("                       PERFORMANCE CHARACTERISTICS", file=file, flush=True)
    print("    Mach      Alt       W      Fn      Fg    Fram     OPR     PSFC ")
    print(" %7.5f  %7.1f %7.3f %7.1f %7.1f %7.1f %7.3f  %7.5f" %summary_data)


    fs_names = ['fc.Fl_O','inlet.Fl_O','duct1.Fl_O','lpc.Fl_O',
                'icduct.Fl_O','hpc_axi.Fl_O','bld25.Fl_O',
                'hpc_centri.Fl_O','bld3.Fl_O','duct6.Fl_O',
                'burner.Fl_O','hpt.Fl_O','duct43.Fl_O','lpt.Fl_O',
                'itduct.Fl_O','pt.Fl_O','duct12.Fl_O','nozzle.Fl_O']
    fs_full_names = [f'{pt}.{fs}' for fs in fs_names]
    pyc.print_flow_station(prob, fs_full_names, file=file)

    comp_names = ['lpc','hpc_axi','hpc_centri']
    comp_full_names = [f'{pt}.{c}' for c in comp_names]
    pyc.print_compressor(prob, comp_full_names, file=file)

    pyc.print_burner(prob, [f'{pt}.burner'])

    turb_names = ['hpt','lpt','pt']
    turb_full_names = [f'{pt}.{t}' for t in turb_names]
    pyc.print_turbine(prob, turb_full_names, file=file)

    noz_names = ['nozzle']
    noz_full_names = [f'{pt}.{n}' for n in noz_names]
    pyc.print_nozzle(prob, noz_full_names, file=file)

    shaft_names = ['hp_shaft','ip_shaft','lp_shaft']
    shaft_full_names = [f'{pt}.{s}' for s in shaft_names]
    pyc.print_shaft(prob, shaft_full_names, file=file)

    bleed_names = ['bld25', 'bld3']
    bleed_full_names = [f'{pt}.{b}' for b in bleed_names]
    pyc.print_bleed(prob, bleed_full_names, file=file)



def viewer_point(prob, file=sys.stdout):
    """
    print a report of all the relevant cycle properties
    """

    summary_data = (prob['fc.Fl_O:stat:MN'], prob['fc.alt'],prob['inlet.Fl_O:stat:W'], 
                    prob['perf.Fn'], prob['perf.Fg'], prob['inlet.F_ram'],
                    prob['perf.OPR'], prob['perf.PSFC'])

    print(file=file, flush=True)
    print(file=file, flush=True)
    print(file=file, flush=True)
    print("----------------------------------------------------------------------------", file=file, flush=True)
    print("                              POINT:", file=file, flush=True)
    print("----------------------------------------------------------------------------", file=file, flush=True)
    print("                       PERFORMANCE CHARACTERISTICS", file=file, flush=True)
    print("    Mach      Alt       W      Fn      Fg    Fram     OPR     PSFC ")
    print(" %7.5f  %7.1f %7.3f %7.1f %7.1f %7.1f %7.3f  %7.5f" %summary_data)


    fs_names = ['fc.Fl_O','inlet.Fl_O','duct1.Fl_O','lpc.Fl_O',
                'icduct.Fl_O','hpc_axi.Fl_O','bld25.Fl_O',
                'hpc_centri.Fl_O','bld3.Fl_O','duct6.Fl_O',
                'burner.Fl_O','hpt.Fl_O','duct43.Fl_O','lpt.Fl_O',
                'itduct.Fl_O','pt.Fl_O','duct12.Fl_O','nozzle.Fl_O']
    fs_full_names = [f'{fs}' for fs in fs_names]
    pyc.print_flow_station(prob, fs_full_names, file=file)

    comp_names = ['lpc','hpc_axi','hpc_centri']
    comp_full_names = [f'{c}' for c in comp_names]
    pyc.print_compressor(prob, comp_full_names, file=file)

    pyc.print_burner(prob, [f'burner'])

    turb_names = ['hpt','lpt','pt']
    turb_full_names = [f'{t}' for t in turb_names]
    pyc.print_turbine(prob, turb_full_names, file=file)

    noz_names = ['nozzle']
    noz_full_names = [f'{n}' for n in noz_names]
    pyc.print_nozzle(prob, noz_full_names, file=file)

    shaft_names = ['hp_shaft','ip_shaft','lp_shaft']
    shaft_full_names = [f'{s}' for s in shaft_names]
    pyc.print_shaft(prob, shaft_full_names, file=file)

    bleed_names = ['bld25', 'bld3']
    bleed_full_names = [f'{b}' for b in bleed_names]
    pyc.print_bleed(prob, bleed_full_names, file=file)


if __name__ == "__main__":

    import time
    from openmdao.api import Problem
    from openmdao.utils.units import convert_units as cu

    prob = om.Problem()

    prob.model = mp_multispool = MultiSpoolTurboshaft()
    prob.setup()


    #Define the design point
    prob.set_val('fc.alt', 11278., units='m')
    prob.set_val('fc.MN', 0.78)
    prob.set_val('fc.MN', 0.78)
    prob.set_val('fc.W', 13.9725, units='kg/s')
    # prob.set_val('balance.rhs:FAR', 1900.0, units='degK'),
    # prob.set_val('balance.rhs:W', 1.09)
    #
    prob.set_val('lpc.PR', 8.000)
    prob.set_val('lpc.eff', 0.8500)
    prob.set_val('hpc_axi.PR', 6.0)
    prob.set_val('hpc_axi.eff', 0.8600)
    prob.set_val('bld25.cool1:frac_W', 0.10)
    prob.set_val('bld25.cool2:frac_W', 0.2)
    # prob.set_val('hpc_centri.PR', 2.0),
    # prob.set_val('hpc_centri.eff', 0.8900),

    prob.set_val('burner.Fl_I:FAR', 0.0396)
    prob.set_val('burner.dPqP', 0.009995)
    prob.set_val('balance.T4_target', 1900.0, units='degK'),
    prob.set_val('hpt.eff', 0.88),
    prob.set_val('hpt.PR', 3.30552517)
    prob.set_val('hpt.cool1:WF', 1)
    prob.set_val('lpt.eff', 0.9),
    prob.set_val('lpt.cool2:WF', 1)
    prob.set_val('pt.eff', 0.9),

    prob.set_val('balance.pwr_target', 8500.0, units='kW')

    # Set initial guesses for balances
    # prob['balance.FAR'] = 0.02261
    # prob['balance.W'] = 13.9
    # prob['balance.hpt_PR'] = 2.5
    # prob['balance.lpt_PR'] = 1.9
    # prob['balance.pt_PR'] = 10.0
    # prob['fc.balance.Pt'] = 5.666
    # prob['fc.balance.Tt'] = 440.0
    #
    # # for i, pt in enumerate(mp_multispool.od_pts):
    # #
    # #     # initial guesses
    # #     prob[pt+'.balance.FAR'] = 0.02135
    # #     prob[pt+'.balance.W'] = 13.9
    # #     prob[pt+'.balance.HP_Nmech'] = 14800.000
    # #     prob[pt+'.balance.IP_Nmech'] = 12000.000
    # #     prob[pt+'.hpt.PR'] = 4.233
    # #     prob[pt+'.lpt.PR'] = 1.979
    # #     prob[pt+'.pt.PR'] = 4.919
    #     prob[pt+'.fc.balance.Pt'] = 5.666
    #     prob[pt+'.fc.balance.Tt'] = 440.0
    #     prob[pt+'.nozzle.PR'] = 1.1

    st = time.time()


    prob.set_solver_print(level=-1)
    prob.set_solver_print(level=2, depth=1)
    prob.run_model()

    # for pt in ['DESIGN']+mp_multispool.od_pts:
    #     viewer(prob, pt)
    # viewer_point(prob)
    print()
    print("time", time.time() - st)
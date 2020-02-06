
#include <vector>
#include <array>
#include <string>
#include <iostream>

#include <ginac/ginac.h>

using namespace GiNaC;


std::array<std::vector<symbol>, 2> create_coeffs(std::string suffix)
{
	std::array<std::vector<symbol>, 2> res;
	res[0].resize(4);
	res[1].resize(4);
	res[0][0] = symbol("ax_" + suffix);
	res[1][0] = symbol("ay_" + suffix);
	res[0][1] = symbol("bx_" + suffix);
	res[1][1] = symbol("by_" + suffix);
	res[0][2] = symbol("cx_" + suffix);
	res[1][2] = symbol("cy_" + suffix);
	res[0][3] = symbol("dx_" + suffix);
	res[1][3] = symbol("dy_" + suffix);
	return res;
}

lst create_spline_equation(const std::array<std::vector<symbol>, 2>& coeff, ex t)
{
	return {coeff[0][0] * pow(t, 3) + coeff[0][1] * pow(t, 2) + coeff[0][2] * t + coeff[0][3],
			coeff[1][0] * pow(t, 3) + coeff[1][1] * pow(t, 2) + coeff[1][2] * t + coeff[1][3]};
}


int main()
{
	symbol t("t");

	symbol p_x("p.x()");
	symbol p_y("p.y()");

	symbol q_x("q.x()");
	symbol q_y("q.y()");

	symbol dp_x("dp.x()");
	symbol dp_y("dp.y()");

	symbol dq_x("dq.x()");
	symbol dq_y("dq.y()");

	symbol ddp_x("ddp.x()");
	symbol ddp_y("ddp.y()");

	std::array<std::vector<symbol>, 2> coeff_0 = create_coeffs("0");
	std::array<std::vector<symbol>, 2> coeff_1 = create_coeffs("1");
	std::array<std::vector<symbol>, 2> coeff_2 = create_coeffs("2");

	lst spline_0 = create_spline_equation(coeff_0, t);
	lst spline_1 = create_spline_equation(coeff_1, t);
	lst spline_2 = create_spline_equation(coeff_2, t);

	std::cout << "spline_0: " << spline_0 << std::endl;
	std::cout << "spline_1: " << spline_1 << std::endl;
	std::cout << "spline_2: " << spline_2 << std::endl;

	lst spline_0_dt = ex_to<lst>(diff(spline_0, t));
	lst spline_1_dt = ex_to<lst>(diff(spline_1, t));
	lst spline_2_dt = ex_to<lst>(diff(spline_2, t));

	lst spline_0_ddt = ex_to<lst>(diff(spline_0_dt, t));
	lst spline_1_ddt = ex_to<lst>(diff(spline_1_dt, t));
	lst spline_2_ddt = ex_to<lst>(diff(spline_2_dt, t));

	lst spline_0_dddt = ex_to<lst>(diff(spline_0_ddt, t));
	lst spline_1_dddt = ex_to<lst>(diff(spline_1_ddt, t));
	lst spline_2_dddt = ex_to<lst>(diff(spline_2_ddt, t));

	lst equations;

	equations.append(spline_0[0].subs(t == 0) == p_x);			// start position
	equations.append(spline_0[1].subs(t == 0) == p_y);			// start position
	equations.append(spline_2[0].subs(t == 1) == q_x);			// end position
	equations.append(spline_2[1].subs(t == 1) == q_y);			// end position
	equations.append(spline_0_dt[0].subs(t == 0) == dp_x);		// start orientation + velocity
	equations.append(spline_0_dt[1].subs(t == 0) == dp_y);		// start orientation + velocity
	equations.append(spline_2_dt[0].subs(t == 1) == dq_x);		// end orientation + velocity
	equations.append(spline_2_dt[1].subs(t == 1) == dq_y);		// end orientation + velocity
	equations.append(spline_0_ddt[0].subs(t == 0) == ddp_x);	// start yawrate + acceleration
	equations.append(spline_0_ddt[1].subs(t == 0) == ddp_y);	// start yawrate + acceleration
	equations.append(spline_2_ddt[0].subs(t == 1) == 0);		// end yawrate + acceleration
	equations.append(spline_2_ddt[1].subs(t == 1) == 0);		// end yawrate + acceleration
	// spline_1_dddt[0].subs(t == 1) == 0,			// end d_yawrate_dt + d_acceleration_dt
	// spline_1_dddt[1].subs(t == 1) == 0,			// end d_yawrate_dt + d_acceleration_dt
	equations.append(spline_0[0].subs(t == 1) == spline_1[0].subs(t == 0));				// position connectivity
	equations.append(spline_0[1].subs(t == 1) == spline_1[1].subs(t == 0));				// position connectivity
	equations.append(spline_0_dt[0].subs(t == 1) == spline_1_dt[0].subs(t == 0));		// orientation + velocity connectivity
	equations.append(spline_0_dt[1].subs(t == 1) == spline_1_dt[1].subs(t == 0));		// orientation + velocity connectivity
	equations.append(spline_0_ddt[0].subs(t == 1) == spline_1_ddt[0].subs(t == 0));		// yawrate + acceleration connectivity
	equations.append(spline_0_ddt[1].subs(t == 1) == spline_1_ddt[1].subs(t == 0));		// yawrate + acceleration connectivity
	equations.append(spline_1[0].subs(t == 1) == spline_2[0].subs(t == 0));				// position connectivity
	equations.append(spline_1[1].subs(t == 1) == spline_2[1].subs(t == 0));				// position connectivity
	equations.append(spline_1_dt[0].subs(t == 1) == spline_2_dt[0].subs(t == 0));		// orientation + velocity connectivity
	equations.append(spline_1_dt[1].subs(t == 1) == spline_2_dt[1].subs(t == 0));		// orientation + velocity connectivity
	equations.append(spline_1_ddt[0].subs(t == 1) == spline_2_ddt[0].subs(t == 0));		// yawrate + acceleration connectivity
	equations.append(spline_1_ddt[1].subs(t == 1) == spline_2_ddt[1].subs(t == 0));		// yawrate + acceleration connectivity

	lst unknowns;

	for(int i = 0; i < 4; ++i) {
		unknowns.append(coeff_0[0][i]);
		unknowns.append(coeff_0[1][i]);
	}
	for(int i = 0; i < 4; ++i) {
		unknowns.append(coeff_1[0][i]);
		unknowns.append(coeff_1[1][i]);
	}
	for(int i = 0; i < 4; ++i) {
		unknowns.append(coeff_2[0][i]);
		unknowns.append(coeff_2[1][i]);
	}

	lst solution = ex_to<lst>(lsolve(equations, unknowns));

	for(int i = 0; i < solution.nops(); ++i) {
		std::cout << solution[i] << std::endl;
	}

	ex result = ex_to<lst>(ex(spline_0).subs(solution));
	ex result_dt = ex_to<lst>(ex(spline_0_dt).subs(solution));
	ex result_ddt = ex_to<lst>(ex(spline_0_ddt).subs(solution));

	std::cout << csrc << "result_x = " << result[0] << std::endl;
	std::cout << csrc << "result_y = " << result[1] << std::endl;
	std::cout << csrc << "result_dx = " << result_dt[0] << std::endl;
	std::cout << csrc << "result_dy = " << result_dt[1] << std::endl;
	std::cout << csrc << "result_ddx = " << result_ddt[0] << std::endl;
	std::cout << csrc << "result_ddy = " << result_ddt[1] << std::endl;

	ex result_1 = ex_to<lst>(ex(spline_1).subs(solution));
	ex result_1_dt = ex_to<lst>(ex(spline_1_dt).subs(solution));

	std::cout << csrc << "result_1_x = " << result_1[0] << std::endl;
	std::cout << csrc << "result_1_y = " << result_1[1] << std::endl;
	std::cout << csrc << "result_1_dx = " << result_1_dt[0] << std::endl;
	std::cout << csrc << "result_1_dy = " << result_1_dt[1] << std::endl;

	ex result_2 = ex_to<lst>(ex(spline_2).subs(solution));
	ex result_2_dt = ex_to<lst>(ex(spline_2_dt).subs(solution));

	std::cout << csrc << "result_2_x = " << result_2[0] << std::endl;
	std::cout << csrc << "result_2_y = " << result_2[1] << std::endl;
	std::cout << csrc << "result_2_dx = " << result_2_dt[0] << std::endl;
	std::cout << csrc << "result_2_dy = " << result_2_dt[1] << std::endl;

}



#include <algorithm>
#include"kaanh.h"
#include"forcecontrol.h"


using namespace aris::dynamic;
using namespace aris::plan;


namespace kaanh
{
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
		#ifdef UNIX
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[0]).setPosOffset(0.00293480352126769);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[1]).setPosOffset(-2.50023777179214);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[2]).setPosOffset(-0.292382537944081);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[3]).setPosOffset(0.0582675097338009);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[4]).setPosOffset(1.53363576057128);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[5]).setPosOffset(26.3545454214145);
        #endif
		return controller;
	};
	auto createModelRokaeXB4(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", aris::core::Matrix(PI));

		// add part //
		auto &p1 = model->partPool().add<Part>("L1");
		auto &p2 = model->partPool().add<Part>("L2");
		auto &p3 = model->partPool().add<Part>("L3");
		auto &p4 = model->partPool().add<Part>("L4");
		auto &p5 = model->partPool().add<Part>("L5");
		auto &p6 = model->partPool().add<Part>("L6");

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, 0.176 };
		const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
		const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
		const double j4_pos[3]{ -0.1233, 0.0, 0.6295, };
		const double j5_pos[3]{ 0.32, -0.03235, 0.6295, };
		const double j6_pos[3]{ 0.383, 0.0, 0.6295, };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		// add ee general motion //
		double pq_ee_i[]{ 0.398, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };		//x方向加上0.1
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// change robot pose //
		if (robot_pm)
		{
			p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
			p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
			p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
			p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
			p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
			p6.setPm(s_pm_dot_pm(robot_pm, *p6.pm()));
			j1.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ().prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();

		inverse_kinematic.setWhichRoot(8);

		return model;
	}


	struct MoveJRParam
	{
		double begin_pos, target_pos, vel, acc, dec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJRParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				param.target_pos = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		target.param = param;
/*
		target.option |=
			Plan::USE_TARGET_POS;
*/
/*
		target.option |=
#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
*/
	}
	auto MoveJR::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRParam&>(target.param);
		auto controller = target.controller;
		
		//获取第一个count时，电机的当前角度位置//
		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_pos = controller->motionAtAbs(i).actualPos();
				}
			}
		}
		
		//梯形轨迹//
		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				target.model->motionPool().at(i).setMp(p);
				total_count = std::max(total_count, t_count);
			}
		}
		//3D模型同步
		if (!target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "target_pos" << ":" << param.target_pos << " ";
			cout << "vel" << ":" << param.vel << " ";
			cout << "acc" << ":" << param.acc << " ";
			cout << "dec"  << ":" << param.dec << " ";
			
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
					cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
					cout << "actualCur" << ":" << controller->motionAtAbs(i).actualCur() << " ";
				}
			}

			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" abbreviation=\"v\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"		<UniqueParam default=\"check_none\">"
			"			<Param name=\"check_all\"/>"
			"			<Param name=\"check_none\"/>"
			"			<GroupParam>"
			"				<UniqueParam default=\"check_pos\">"
			"					<Param name=\"check_pos\"/>"
			"					<Param name=\"not_check_pos\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_pos_max\">"
			"							<Param name=\"check_pos_max\"/>"
			"							<Param name=\"not_check_pos_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_min\">"
			"							<Param name=\"check_pos_min\"/>"
			"							<Param name=\"not_check_pos_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous\">"
			"							<Param name=\"check_pos_continuous\"/>"
			"							<Param name=\"not_check_pos_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_at_start\">"
			"							<Param name=\"check_pos_continuous_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order\">"
			"							<Param name=\"check_pos_continuous_second_order\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
			"							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_following_error\">"
			"							<Param name=\"check_pos_following_error\"/>"
			"							<Param name=\"not_check_pos_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"				<UniqueParam default=\"check_vel\">"
			"					<Param name=\"check_vel\"/>"
			"					<Param name=\"not_check_vel\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_vel_max\">"
			"							<Param name=\"check_vel_max\"/>"
			"							<Param name=\"not_check_vel_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_min\">"
			"							<Param name=\"check_vel_min\"/>"
			"							<Param name=\"not_check_vel_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous\">"
			"							<Param name=\"check_vel_continuous\"/>"
			"							<Param name=\"not_check_vel_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous_at_start\">"
			"							<Param name=\"check_vel_continuous_at_start\"/>"
			"							<Param name=\"not_check_vel_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_following_error\">"
			"							<Param name=\"check_vel_following_error\"/>"
			"							<Param name=\"not_check_vel_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}

	struct MoveSineParam
	{
		double begin_pos, target_pos, vel, acc, dec, amp, cycle, phi0, offset;
			int total_time;//持续时间，默认5000count
		std::vector<bool> joint_active_vec;
	};
	auto MoveSine::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveSineParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				param.target_pos = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "cycle")
			{
				param.cycle = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "amp")
			{
				param.amp = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "phi0")
			{
				param.phi0 = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "offset")
			{
				param.offset = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "total_time")
			{
				param.total_time = std::stoi(cmd_param.second);
			}
		}

		target.param = param;
		/*
				target.option |=
					Plan::USE_TARGET_POS;
		*/
		/*
				target.option |=
		#ifdef WIN32
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		#endif
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
		*/
	}
	auto MoveSine::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveSineParam&>(target.param);
		auto controller = target.controller;

		//获取第一个count时，电机的当前角度位置//
		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_pos = controller->motionAtAbs(i).actualPos();
				}
			}
		}

		//梯形轨迹//
		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				
				aris::Size t_count;
				param.target_pos = param.begin_pos+param.amp*(std::sin(1.0*target.count / param.cycle * 2 * aris::PI))+param.offset;
				controller->motionAtAbs(i).setTargetPos(param.target_pos);
				
				/*aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);*/
				/*controller->motionAtAbs(i).setTargetPos(p);*/
				target.model->motionPool().at(i).setMp(param.target_pos);
			/*	total_count = std::max(total_count, t_count);*/
			}
		}
		//3D模型同步
		if (!target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "target_pos" << ":" << param.target_pos << " ";
			//cout << "vel" << ":" << param.vel << " ";
			//cout << "acc" << ":" << param.acc << " ";
			//cout << "dec" << ":" << param.dec << " ";

			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
					cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
					cout << "actualCur" << ":" << controller->motionAtAbs(i).actualCur() << " ";
				}
			}

			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return param.total_time - target.count;
	}
	auto MoveSine::collectNrt(PlanTarget &target)->void {}
	MoveSine::MoveSine(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveSine\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" abbreviation=\"v\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"		<Param name=\"amp\" default=\"1\"/>"
			"		<Param name=\"cycle\" default=\"1000\"/>"
			"		<Param name=\"phi0\" default=\"0\"/>"
			"		<Param name=\"offset\" default=\"0\"/>"
			"		<Param name=\"total_time\" default=\"5000\"/>"
			"		<UniqueParam default=\"check_none\">"
			"			<Param name=\"check_all\"/>"
			"			<Param name=\"check_none\"/>"
			"			<GroupParam>"
			"				<UniqueParam default=\"check_pos\">"
			"					<Param name=\"check_pos\"/>"
			"					<Param name=\"not_check_pos\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_pos_max\">"
			"							<Param name=\"check_pos_max\"/>"
			"							<Param name=\"not_check_pos_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_min\">"
			"							<Param name=\"check_pos_min\"/>"
			"							<Param name=\"not_check_pos_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous\">"
			"							<Param name=\"check_pos_continuous\"/>"
			"							<Param name=\"not_check_pos_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_at_start\">"
			"							<Param name=\"check_pos_continuous_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order\">"
			"							<Param name=\"check_pos_continuous_second_order\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
			"							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_following_error\">"
			"							<Param name=\"check_pos_following_error\"/>"
			"							<Param name=\"not_check_pos_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"				<UniqueParam default=\"check_vel\">"
			"					<Param name=\"check_vel\"/>"
			"					<Param name=\"not_check_vel\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_vel_max\">"
			"							<Param name=\"check_vel_max\"/>"
			"							<Param name=\"not_check_vel_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_min\">"
			"							<Param name=\"check_vel_min\"/>"
			"							<Param name=\"not_check_vel_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous\">"
			"							<Param name=\"check_vel_continuous\"/>"
			"							<Param name=\"not_check_vel_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous_at_start\">"
			"							<Param name=\"check_vel_continuous_at_start\"/>"
			"							<Param name=\"not_check_vel_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_following_error\">"
			"							<Param name=\"check_vel_following_error\"/>"
			"							<Param name=\"not_check_vel_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}

	

	// 示教运动--输入末端大地坐标系的位姿pe，控制动作 //
	struct MovePointParam
	{
		std::vector<double> term_begin_pe_vec;
		std::vector<double> begin_pm;
		std::vector<double> target_pm;
		aris::Size cor;
		aris::Size move_type;
		double x, y, z, a, b, c, vel, acc, dec, term_offset_pe;
	};
	auto MovePoint::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MovePointParam param;
		param.term_begin_pe_vec.resize(6, 0.0);
		param.begin_pm.resize(16, 0.0);
		param.term_offset_pe = 0;
		param.target_pm.resize(16, 0.0);

		std::string ret = "ok";
		target.ret = ret;

		for (auto &p : params)
		{
			if (p.first == "cor")
			{
				param.cor = std::stoi(p.second);
			}
			else if (p.first == "x")
			{
				param.x = std::stod(p.second);
				param.move_type = 0;
				param.term_offset_pe = param.x;
			}
			else if (p.first == "y")
			{
				param.y = std::stod(p.second);
				param.move_type = 1;
				param.term_offset_pe = param.y;
			}
			else if (p.first == "z")
			{
				param.z = std::stod(p.second);
				param.move_type = 2;
				param.term_offset_pe = param.z;
			}
			else if (p.first == "a")
			{
				param.a = std::stod(p.second);
				param.move_type = 3;
				param.term_offset_pe = param.a;
			}
			else if (p.first == "b")
			{
				param.b = std::stod(p.second);
				param.move_type = 4;
				param.term_offset_pe = param.b;
			}
			else if (p.first == "c")
			{
				param.c = std::stod(p.second);
				param.move_type = 5;
				param.term_offset_pe = param.c;
			}
			else if (p.first == "vel")
			{
				param.vel = std::stod(p.second);
			}
			else if (p.first == "acc")
			{
				param.acc = std::stod(p.second);
			}
			else if (p.first == "dec")
			{
				param.dec = std::stod(p.second);
			}
		}
		target.param = param;

		target.option |=
			Plan::USE_TARGET_POS |
#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto MovePoint::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<MovePointParam&>(target.param);
		static aris::Size total_count = 1;

		char eu_type[4]{ '1', '2', '3', '\0' };

		if (target.count == 1)
		{
			// 获取起始欧拉角位姿 //
			target.model->generalMotionPool().at(0).getMpe(param.term_begin_pe_vec.data(), eu_type);
		}
		// 梯形轨迹规划 //
		double p, v, a;
		aris::Size t_count;
		aris::plan::moveAbsolute(target.count, 0, param.term_offset_pe, param.vel / 1000
			, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
		total_count = std::max(total_count, t_count);

		double pe[6]{ 0,0,0,0,0,0 }, pm[16];
		pe[param.move_type] = p;
		s_pe2pm(pe, pm, eu_type);

		s_pe2pm(param.term_begin_pe_vec.data(), param.begin_pm.data(), eu_type);

		//绝对坐标系
		if (param.cor == 0)
		{
			s_pm_dot_pm(pm, param.begin_pm.data(), param.target_pm.data());
		}
		//工件坐标系
		else if (param.cor == 1)
		{
			s_pm_dot_pm(param.begin_pm.data(), pm, param.target_pm.data());
		}
		target.model->generalMotionPool().at(0).setMpm(param.target_pm.data());


		// 运动学反解 //
		if (!target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();

		if (target.count % 200 == 0)
		{
			for (Size i = 0; i < 16; i++)
			{
				cout << param.target_pm[i] << "  ";
			}
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
			lout << controller->motionAtAbs(i).actualCur() << " ";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MovePoint::collectNrt(PlanTarget &target)->void {}
	MovePoint::MovePoint(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"movePoint\">"
			"	<GroupParam>"
			"		<Param name=\"cor\" default=\"0\"/>"
			"		<Param name=\"vel\" default=\"0.2\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"0.4\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"0.4\" abbreviation=\"d\"/>"
			"		<UniqueParam default=\"x\">"
			"			<Param name=\"x\" default=\"0.02\"/>"
			"			<Param name=\"y\" default=\"0.02\"/>"
			"			<Param name=\"z\" default=\"0.02\"/>"
			"			<Param name=\"a\" default=\"0.17\"/>"
			"			<Param name=\"b\" default=\"0.17\"/>"
			"			<Param name=\"c\" default=\"0.17\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}



	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(aris::robot::createPlanRootRokaeXB4());

		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<kaanh::MoveJR>();
		plan_root->planPool().add<kaanh::MoveSine>();
		plan_root->planPool().add<kaanh::MovePoint>();
		plan_root->planPool().add<forcecontrol::MoveJRC>();


		return plan_root;
	}
	
}

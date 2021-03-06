///*
// * carmenu.h
// *
// *  Created on: 27 Jan, 2015
// *      Author: lincoln
// */
//
//#ifndef SRC_CARMENU_H_
//#define SRC_CARMENU_H_
//
//#include <cstdint>
//
//#include <libutil/remote_var_manager.h>
//
//namespace linear_ccd
//{
//class Car ;
//
//}
//
//namespace linear_ccd
//{
//
//class carmenu
//{
//public:
//	explicit carmenu(Car *const car);
//
//	void Run();
//
//	uint32_t GetCcdThreshold() const
//	{
//		return m_ccd_threshold->GetInt();
//	}
//
//	uint32_t GetMid() const
//	{
//		return m_mid->GetInt();
//	}
//
//	uint32_t GetEdge() const
//	{
//		return m_edge->GetInt();
//	}
//
//	float GetTurnKp() const
//	{
//		return m_turn_kp->GetReal();
//	}
//
//	uint32_t GetTurnKpFn() const
//	{
//		return m_turn_kp_fn->GetInt();
//	}
//
//	float GetTurnKd() const
//	{
//		return m_turn_kd->GetReal();
//	}
//
//	uint32_t GetTurnKdFn() const
//	{
//		return m_turn_kd_fn->GetInt();
//	}
//
//	uint32_t GetSpeedSp() const
//	{
//		return m_speed_sp->GetInt();
//	}
//
//	float GetSpeedKp() const
//	{
//		return m_speed_kp->GetReal();
//	}
//
//	float GetSpeedKi() const
//	{
//		return m_speed_ki->GetReal();
//	}
//
//	float GetSpeedKd() const
//	{
//		return m_speed_kd->GetReal();
//	}
//
//	uint32_t GetSpeedTurnSp() const
//	{
//		return m_speed_turn_sp->GetInt();
//	}
//
//private:
//	enum Page
//	{
//		TURN,
//		SPEED,
//
//		SIZE,
//	};
//
//	static constexpr int TUNABLE_INT_COUNT = 12;
//
//	void Select(const int id);
//	void SwitchPage(const int id);
//
//	int GetMultiplier() const;
//
//	void AdjustValueTurn(const bool is_positive);
//	void AdjustValueSpeed(const bool is_positive);
//
//	void Redraw(const bool is_clear_screen);
//
//	Car *m_car;
//
//	Page m_page;
//	int m_select;
//
//	libutil::RemoteVarManager::Var *m_ccd_threshold;
//	libutil::RemoteVarManager::Var *m_mid;
//	libutil::RemoteVarManager::Var *m_edge;
//	libutil::RemoteVarManager::Var *m_turn_kp;
//	libutil::RemoteVarManager::Var *m_turn_kp_fn;
//	libutil::RemoteVarManager::Var *m_turn_kd;
//	libutil::RemoteVarManager::Var *m_turn_kd_fn;
//
//	libutil::RemoteVarManager::Var *m_speed_sp;
//	libutil::RemoteVarManager::Var *m_speed_kp;
//	libutil::RemoteVarManager::Var *m_speed_ki;
//	libutil::RemoteVarManager::Var *m_speed_kd;
//	libutil::RemoteVarManager::Var *m_speed_turn_sp;
//};
//
//}
//
//
//#endif /* SRC_CARMENU_H_ */

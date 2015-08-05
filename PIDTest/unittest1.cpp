#include "stdafx.h"
#include "CppUnitTest.h"
#include "../pid.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PIDTest
{		
	TEST_CLASS(PIDTest)
	{
	public:
		
		TEST_METHOD(UpperLimiter_Function_Test_1)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, onlyUpperLimiter);
			SetPIDUpperLimit(&testPID, 2000);
			double result = OutputLimiter(&testPID, 3000);
			Assert::AreEqual(2000.0, result, L"PID Upper Limiter can't restrict a large output.");
		}

		TEST_METHOD(UpperLimiter_Function_Test_2)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, onlyUpperLimiter);
			SetPIDUpperLimit(&testPID, 2000);
			double result = OutputLimiter(&testPID, 1000);
			Assert::AreEqual(1000.0, result, L"PID Upper Limiter is malfunction");
		}

		TEST_METHOD(Lower_Limiter_Function_Test_1)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, onlyLowerLimiter);
			SetPIDLowerLimit(&testPID, -100);
			double result = OutputLimiter(&testPID, -300);
			Assert::AreEqual(-100.0, result, L"PID Lower Limiter can't restrict a small output");
		}

		TEST_METHOD(Lower_Limiter_Function_Test_2)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, onlyLowerLimiter);
			SetPIDLowerLimit(&testPID, -100);
			double result = OutputLimiter(&testPID, 0);
			Assert::AreEqual(0.0, result, L"PID Lower Limiter is malfunction");
		}

		TEST_METHOD(Limiter_Function_Test_1)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, both);
			SetPIDUpperLimit(&testPID, 200);
			SetPIDLowerLimit(&testPID, -100);
			double result = OutputLimiter(&testPID, 0);
			Assert::AreEqual(0.0, result, L"Limiter is malfunction");
		}

		TEST_METHOD(Limiter_Function_Test_2)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, both);
			SetPIDUpperLimit(&testPID, 200);
			SetPIDLowerLimit(&testPID, -100);
			double result = OutputLimiter(&testPID, 300);
			Assert::AreEqual(200.0, result, L"Limiter is malfunction");
		}

		TEST_METHOD(Limiter_Function_Test_3)
		{
			PID_t testPID;
			ConfigurePIDLimiter(&testPID, both);
			SetPIDUpperLimit(&testPID, 200);
			SetPIDLowerLimit(&testPID, -100);
			double result = OutputLimiter(&testPID, -200);
			Assert::AreEqual(-100.0, result, L"PID Lower Limiter is malfunction");
		}
	};
}
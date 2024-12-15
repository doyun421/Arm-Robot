 Jacobian /Inverse Jacobian

// 야코비안을 계산하는 함수
TArray<FRobotArmData> ARobotArmSimulation::CalculateJacobian(const TArray<FRobotArmData>& dataset)
{
    TArray<FRobotArmData> resultDataset = dataset; // 결과를 저장할 데이터셋
    int numData = dataset.Num();  // 데이터셋의 크기
    int numJoints = 6;  // 관절의 개수 (theta1부터 theta6까지)

    TArray<FMatrix> Jacobians; // 야코비안 행렬을 저장할 배열

    // 각 데이터에 대해 야코비안의 각 열을 계산
    for (int i = 0; i < numData - 1; ++i)
    {
        // 현재 데이터와 다음 데이터를 가져옴
        FRobotArmData& currentData = resultDataset[i];
        const FRobotArmData& nextData = dataset[i + 1];

        // 현재와 다음 End Effector 위치 계산 (ΔX, ΔY, ΔZ)
        float dX = nextData.EndEffector.X - currentData.EndEffector.X;
        float dY = nextData.EndEffector.Y - currentData.EndEffector.Y;
        float dZ = nextData.EndEffector.Z - currentData.EndEffector.Z;

        // 현재 각 관절 각도 (theta1부터 theta6까지) 계산
        TArray<float> currentAngles;
        currentAngles.Add(currentData.theta1);
        currentAngles.Add(currentData.theta2);
        currentAngles.Add(currentData.theta3);
        currentAngles.Add(currentData.theta4);
        currentAngles.Add(currentData.theta5);
        currentAngles.Add(currentData.theta6);

        TArray<float> nextAngles;
        nextAngles.Add(nextData.theta1);
        nextAngles.Add(nextData.theta2);
        nextAngles.Add(nextData.theta3);
        nextAngles.Add(nextData.theta4);
        nextAngles.Add(nextData.theta5);
        nextAngles.Add(nextData.theta6);

        // 각 관절 변화량 계산 (Δtheta)
        TArray<float> dTheta;
        for (int k = 0; k < currentAngles.Num(); ++k)
        {
            dTheta.Add(nextAngles[k] - currentAngles[k]);
        }

        // 야코비안 행렬의 각 열을 계산
        FMatrix JacobianMatrix = FMatrix::Identity;

        for (int j = 0; j < numJoints; ++j)
        {
            // 각 관절에 대한 X, Y, Z 변화량을 반영하여 Jacobian 행렬 계산
            if (dTheta[j] != 0)  // Δtheta가 0이 아닐 때만 계산
            {
                JacobianMatrix.M[0][j] = dX / dTheta[j]; // X축 변화량
                JacobianMatrix.M[1][j] = dY / dTheta[j]; // Y축 변화량
                JacobianMatrix.M[2][j] = dZ / dTheta[j]; // Z축 변화량
            }
            else
            {
                // Δtheta가 0일 경우에는 특정 값 처리 (예: 0으로 설정)
                JacobianMatrix.M[0][j] = 0;
                JacobianMatrix.M[1][j] = 0;
                JacobianMatrix.M[2][j] = 0;
            }
        }

        // 계산된 Jacobian 행렬을 resultDataset의 해당 데이터에 저장
        currentData.JacobianMatrix = JacobianMatrix;

        // Jacobian 행렬을 별도의 배열에 추가
        Jacobians.Add(JacobianMatrix);
    }

    // Log the Jacobian matrices in a readable format
    for (int i = 0; i < Jacobians.Num(); ++i)
    {
        FMatrix& JacobianMatrix = Jacobians[i];
        for (int row = 0; row < 3; ++row)
        {
            FString RowString = FString::Printf(TEXT("Row %d: "), row);
            for (int col = 0; col < 6; ++col)
            {
                RowString += FString::Printf(TEXT("%f "), JacobianMatrix.M[row][col]);
            }
            UE_LOG(LogTemp, Warning, TEXT("%s"), *RowString);
        }
    }

    return resultDataset;
}




FVector ARobotArmSimulation::CalculateEndEffectorPosition(const FRobotArmData& data)
{
    // 링크 길이 (단위: 미터)
    const double L12 = 0.473488;  // Base to Shoulder
    const double L23 = 0.847927;  // Shoulder to Elbow
    const double L34 = 0.481544;  // Elbow to Wrist
    const double L45 = 0.458416;  // Wrist to End of Wrist
    const double L56 = 0.03133;   // End of Wrist to End Effector

    // 각 관절 각도 (theta1부터 theta6까지)
    double theta1 = FMath::DegreesToRadians(data.theta1); // 각도를 라디안으로 변환
    double theta2 = FMath::DegreesToRadians(data.theta2);
    double theta3 = FMath::DegreesToRadians(data.theta3);
    double theta4 = FMath::DegreesToRadians(data.theta4);
    double theta5 = FMath::DegreesToRadians(data.theta5);
    double theta6 = FMath::DegreesToRadians(data.theta6);

    // Base -> Shoulder 위치 계산 (X축 회전, theta1)
    FRotator rot1(0, 0, theta1);  // Z축 회전
    FVector shoulderPosition = rot1.RotateVector(FVector(L12, 0, 0));

    // Shoulder -> Elbow 위치 계산 (Y축 회전, theta2)
    FRotator rot2(0, theta2, 0);  // Y축 회전
    FVector elbowPosition = shoulderPosition + rot2.RotateVector(FVector(L23, 0, 0));

    // Elbow -> Wrist 위치 계산 (Y축 회전, theta3)
    FRotator rot3(0, theta3, 0);  // Y축 회전
    FVector wristPosition = elbowPosition + rot3.RotateVector(FVector(L34, 0, 0));

    // Wrist -> Wrist End 위치 계산 (Z축 회전, theta4)
    FRotator rot4(0, 0, theta4);  // Z축 회전
    FVector wristEndPosition = wristPosition + rot4.RotateVector(FVector(L45, 0, 0));

    // Wrist End -> Final Wrist 위치 계산 (X축 회전, theta5)
    FRotator rot5(theta5, 0, 0);  // X축 회전
    FVector finalPosition = wristEndPosition + rot5.RotateVector(FVector(L56, 0, 0));

    // Final Wrist -> End Effector 위치 계산 (X축 회전, theta6)
    FRotator rot6(theta6, 0, 0);  // X축 회전
    FVector endEffectorPosition = finalPosition + rot6.RotateVector(FVector(0, 0, 0)); // 이 경우 실제로는 위치 변화가 없을 수 있음

    // 최종 엔드 이펙터 위치 반환
    return endEffectorPosition;
}





TArray<FRobotArmData> ARobotArmSimulation::CalculateInverseJacobian(const FRobotArmData &currentData, const FVector& targetEndEffector)
{
    TArray<FRobotArmData> resultDataset;

    // 6x6 역야코비안 행렬을 구할 변수
    FMatrix JacobianMatrix = currentData.JacobianMatrix; // 기존에 계산된 야코비안
    FMatrix JacobianInverse = JacobianMatrix.Inverse(); // 역행렬 구하기

    // 목표 엔드 이펙터 위치와 현재 엔드 이펙터 위치 간의 차이 (Δx)
    FVector deltaX = targetEndEffector - currentData.EndEffector;

    // Δθ (각도 변화량)를 구하기 위해 역야코비안 * Δx 계산
    TArray<float> deltaTheta;

    // 3D 위치 변화 (X, Y, Z)를 계산하여 각 관절의 변화량을 구함
    for (int i = 0; i < 6; ++i)
    {
        float dTheta = 0.0f;
        for (int row = 0; row < 3; ++row)
        {
            dTheta += JacobianInverse.M[row][i] * deltaX[row]; // 각도 변화량 계산
        }
        deltaTheta.Add(dTheta);
    }

    // 각 관절의 새로운 각도를 계산 (현재 각도 + Δθ)
    FRobotArmData newData = currentData;
    newData.theta1 += deltaTheta[0];
    newData.theta2 += deltaTheta[1];
    newData.theta3 += deltaTheta[2];
    newData.theta4 += deltaTheta[3];
    newData.theta5 += deltaTheta[4];
    newData.theta6 += deltaTheta[5];

    // 새로운 데이터셋에 추가
    resultDataset.Add(newData);

    // 새로 계산된 각도를 기반으로 엔드 이펙터의 위치를 업데이트
    newData.EndEffector = CalculateEndEffectorPosition(newData); // 새로 계산된 관절 각도를 통해 엔드 이펙터 위치 계산

    // 엔드 이펙터의 위치를 콘솔에 로그로 출력
    FString resultLog = FString::Printf(TEXT("New Data - EndEffector Position: (X: %.3f, Y: %.3f, Z: %.3f), Angles: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f"),
        newData.EndEffector.X, newData.EndEffector.Y, newData.EndEffector.Z,
        newData.theta1, newData.theta2, newData.theta3, newData.theta4, newData.theta5, newData.theta6);

    UE_LOG(LogTemp, Warning, TEXT("%s"), *resultLog);  // 콘솔에 출력

    // 결과 반환
    return resultDataset;
}

#include <Novice.h>
#include <MyMath.h>
#include <imgui.h>

const char kWindowTitle[] = "LE2A_08_スヤマハナ";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	float kWindowWidth = 1280;
	float kWindowHeight = 720;

	Vector3 rotate{ 0,5,0 };
	Vector3 translate{};
	Vector3 cameraPosition{ 0, 0, -5 };

	AABB aabb{
		.min{-0.5f, -0.5f, -0.5},
		.max{0.0f, 0.0f, 0.0f},
	};

	Sphere sphere{ {0,0,0}, 2 };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		//各種行列の計算
		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, rotate, translate);
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, cameraPosition);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 worldViewProjectionMatrix = Multiply(worldMatrix, Multiply(viewMatrix, projectionMatrix));
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		ImGui::Begin("ImGui");
		ImGui::SliderFloat3("AABB1 Max", reinterpret_cast<float*>(&aabb.max), -5, 5);
		ImGui::SliderFloat3("AABB1 Min", reinterpret_cast<float*>(&aabb.min), -5, 5);
		ImGui::SliderFloat3("Sphere Center", reinterpret_cast<float*>(&sphere.center), -5, 5);
		ImGui::SliderFloat("Sphere Radius", reinterpret_cast<float*>(&sphere.radius), -5, 5);
		ImGui::SliderFloat3("Camera Rotate", reinterpret_cast<float*>(&rotate), -3, 3);
		ImGui::SliderFloat3("Camera Translate", reinterpret_cast<float*>(&translate), -3, 3);
		ImGui::End();

		aabb.min.x = (std::min)(aabb.min.x, aabb.max.x);
		aabb.max.x = (std::max)(aabb.min.x, aabb.max.x);
		aabb.min.y = (std::min)(aabb.min.y, aabb.max.y);
		aabb.max.y = (std::max)(aabb.min.y, aabb.max.y);
		aabb.min.z = (std::min)(aabb.min.z, aabb.max.z);
		aabb.max.z = (std::max)(aabb.min.z, aabb.max.z);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		DrawGrid(worldViewProjectionMatrix, viewportMatrix);

		if (IsCollision(aabb, sphere)) {
			DrawAABB(aabb, worldViewProjectionMatrix, viewportMatrix, RED);
		} else {
			DrawAABB(aabb, worldViewProjectionMatrix, viewportMatrix, WHITE);
		}

		DrawSphere(sphere, worldViewProjectionMatrix, viewportMatrix, WHITE);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}

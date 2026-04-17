#include <Novice.h>
#include <math.h>

struct Vector {
	float x;
	float y;
	float z;
};

// 加算
Vector Add(Vector v1, Vector v2) { return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }

// 減算
Vector Subtract(Vector v1, Vector v2) { return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z}; }

// スカラー倍
Vector Multiply(float k, Vector v) { return {k * v.x, k * v.y, k * v.z}; }

// 内積
float Dot(Vector v1, Vector v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }

// 長さ
float Length(Vector v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

// 正規化
Vector Normalize(Vector v) {
	float len = Length(v);
	if (len == 0.0f)
		return {0, 0, 0};
	return {v.x / len, v.y / len, v.z / len};
}

void VectorScreenPrintf(int x, int y, Vector v, const char* label) {
    Novice::ScreenPrintf(x, y, "%6.2f %6.2f %6.2f : %s", v.x, v.y, v.z, label);
}
const char kWindowTitle[] = "LC1D_28_ワタナベ_アヤト_タイトル";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

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
		Vector v1 = {1.0f, 3.0f, -5.0f};
		Vector v2 = {4.0f, -1.0f, 2.0f};
		float k = 4.0f;

		Vector resultAdd = Add(v1, v2);
		Vector resultSub = Subtract(v1, v2);
		Vector resultMul = Multiply(k, v1);
		float resultDot = Dot(v1, v2);
		float resultLen = Length(v1);
		Vector resultNor = Normalize(v2);
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		int y = 0;

		VectorScreenPrintf(0, y, resultAdd, "Add");
		y += 20;
		VectorScreenPrintf(0, y, resultSub, "Subtract");
		y += 20;
		VectorScreenPrintf(0, y, resultMul, "Multiply");
		y += 20;

		Novice::ScreenPrintf(0, y, "%6.2f : Dot", resultDot);
		y += 20;
		Novice::ScreenPrintf(0, y, "%6.2f : Length", resultLen);
		y += 20;

		VectorScreenPrintf(0, y, resultNor, "Normalize");
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

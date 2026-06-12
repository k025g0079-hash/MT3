#include <Novice.h>
#include <math.h>
#include <assert.h>
#include <utility> 

struct Vector {
	float x;
	float y;
	float z;
};

struct Matrix4x4
{
	float m[4][4];
};

// 先行宣言
Matrix4x4 MakeIdentity();
Matrix4x4 Multiply(Matrix4x4 A, Matrix4x4 B);

// 加算
Vector Add(Vector v1, Vector v2) { return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z }; }

// 減算
Vector Subtract(Vector v1, Vector v2) { return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z }; }

// スカラー倍
Vector Multiply(float k, Vector v) { return { k * v.x, k * v.y, k * v.z }; }

// 内積
float Dot(Vector v1, Vector v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }

// 長さ
float Length(Vector v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

// 正規化
Vector Normalize(Vector v) {
	float len = Length(v);
	if (len == 0.0f)
		return { 0, 0, 0 };
	return { v.x / len, v.y / len, v.z / len };
}

// ーーー 追加：クロス積（外積）の計算関数 ーーー
Vector Cross(Vector v1, Vector v2) {
	return {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

// ーーー 追加：座標変換（4x4行列によるベクトルの変換、w除算を含む） ーーー
Vector Transform(Vector vector, Matrix4x4 matrix) {
	Vector result;
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];
	result.x = (vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0]) / w;
	result.y = (vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1]) / w;
	result.z = (vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2]) / w;
	return result;
}

// ーーー 追加：透視投影行列の作成 ーーー
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspect, float nearClip, float farClip) {
	Matrix4x4 result = {};
	float cot = 1.0f / tanf(fovY / 2.0f);
	result.m[0][0] = cot / aspect;
	result.m[1][1] = cot;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1.0f;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	return result;
}

// ーーー 追加：ビューポート変換行列の作成 ーーー
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result = MakeIdentity();
	result.m[0][0] = width / 2.0f;
	result.m[1][1] = -height / 2.0f;
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + width / 2.0f;
	result.m[3][1] = top + height / 2.0f;
	result.m[3][2] = minDepth;
	return result;
}

Matrix4x4 m1 = { {
	{3.2f, 0.7f, 9.6f, 4.4f},
	{5.5f, 1.3f, 7.8f, 2.1f},
	{6.9f, 8.0f, 2.6f, 1.0f},
	{0.5f, 7.2f, 5.1f, 3.3f}
} };

Matrix4x4 m2 = { {
	{4.1f, 6.5f, 3.3f, 2.2f},
	{8.8f, 0.6f, 9.9f, 7.7f},
	{1.1f, 5.5f, 6.0f, 0.0f},
	{3.3f, 9.9f, 8.8f, 2.2f}
} };

Matrix4x4 MakeScaleMatrix(Vector scale) {
	Matrix4x4 result = MakeIdentity();
	result.m[0][0] = scale.x;
	result.m[1][1] = scale.y;
	result.m[2][2] = scale.z;
	return result;
}

Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 result = MakeIdentity();
	result.m[1][1] = cosf(radian);
	result.m[1][2] = sinf(radian);
	result.m[2][1] = -sinf(radian);
	result.m[2][2] = cosf(radian);
	return result;
}

Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 result = MakeIdentity();
	result.m[0][0] = cosf(radian);
	result.m[0][2] = -sinf(radian);
	result.m[2][0] = sinf(radian);
	result.m[2][2] = cosf(radian);
	return result;
}

Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 result = MakeIdentity();
	result.m[0][0] = cosf(radian);
	result.m[0][1] = sinf(radian);
	result.m[1][0] = -sinf(radian);
	result.m[1][1] = cosf(radian);
	return result;
}

Matrix4x4 MakeTranslateMatrix(Vector translate) {
	Matrix4x4 result = MakeIdentity();
	result.m[3][0] = translate.x;
	result.m[3][1] = translate.y;
	result.m[3][2] = translate.z;
	return result;
}

Matrix4x4 Multiply(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = 0.0f;
			for (int k = 0; k < 4; k++) {
				result.m[i][j] += A.m[i][k] * B.m[k][j];
			}
		}
	}
	return result;
}

Matrix4x4 MakeAffineMatrix(Vector scale, Vector rotate, Vector translate) {
	Matrix4x4 S = MakeScaleMatrix(scale);
	Matrix4x4 Rx = MakeRotateXMatrix(rotate.x);
	Matrix4x4 Ry = MakeRotateYMatrix(rotate.y);
	Matrix4x4 Rz = MakeRotateZMatrix(rotate.z);
	Matrix4x4 R = Multiply(Multiply(Rx, Ry), Rz);
	Matrix4x4 T = MakeTranslateMatrix(translate);
	return Multiply(Multiply(S, R), T);
}

Matrix4x4 Add(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = A.m[i][j] + B.m[i][j];
		}
	}
	return result;
}

Matrix4x4 Subtract(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = A.m[i][j] - B.m[i][j];
		}
	}
	return result;
}

Matrix4x4 Inverse(Matrix4x4 m) {
	Matrix4x4 result{};
	float a[4][8] = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			a[i][j] = m.m[i][j];
			a[i][j + 4] = (i == j) ? 1.0f : 0.0f;
		}
	}
	for (int i = 0; i < 4; i++) {
		int pivotRow = i;
		float maxVal = fabsf(a[i][i]);
		for (int k = i + 1; k < 4; k++) {
			if (fabsf(a[k][i]) > maxVal) {
				maxVal = fabsf(a[k][i]);
				pivotRow = k;
			}
		}
		if (pivotRow != i) {
			for (int j = 0; j < 8; j++) {
				std::swap(a[i][j], a[pivotRow][j]);
			}
		}
		float pivot = a[i][i];
		assert(fabsf(pivot) > 1e-6f);
		for (int j = 0; j < 8; j++) {
			a[i][j] /= pivot;
		}
		for (int k = 0; k < 4; k++) {
			if (k == i) continue;
			float factor = a[k][i];
			for (int j = 0; j < 8; j++) {
				a[k][j] -= factor * a[i][j];
			}
		}
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = a[i][j + 4];
		}
	}
	return result;
}

Matrix4x4 Transpose(Matrix4x4 m) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = m.m[j][i];
		}
	}
	return result;
}

Matrix4x4 MakeIdentity() {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		result.m[i][i] = 1.0f;
	}
	return result;
}

const char kWindowTitle[] = "LC1D_28_ワタナベ_アヤト_タイトル";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化 (1280x720)
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// ーーー 追加：トランスフォーム変数の初期化 ーーー
	Vector scale = { 1.0f, 1.0f, 1.0f };
	Vector rotate = { 0.0f, 0.0f, 0.0f };
	Vector translate = { 0.0f, 0.0f, 0.0f };

	// カメラ設定
	Vector cameraTranslate = { 0.0f, 0.0f, -5.0f };

	// ローカル空間上の三角形の3頂点
	Vector localVertices[3] = {
		{  0.0f,  1.0f, 0.0f }, // 頂点0 (上)
		{  1.0f, -1.0f, 0.0f }, // 頂点1 (右下)
		{ -1.0f, -1.0f, 0.0f }  // 頂点2 (左下)
	};

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

		// ーーー 追加：キー入力による移動処理（WASD等による平行移動） ーーー
		float speed = 0.05f;
		if (keys[DIK_A]) { translate.x -= speed; }
		if (keys[DIK_D]) { translate.x += speed; }
		if (keys[DIK_W]) { translate.y += speed; }
		if (keys[DIK_S]) { translate.y -= speed; }

		// ーーー 追加：自動回転処理（Y軸周りを毎フレーム一定速度で回転） ーーー
		rotate.y += 0.03f;

		// ーーー 各種行列の合成（レンダリングパイプラインの構築） ーーー
		// 1. World行列の生成 (アフィン変換)
		Matrix4x4 worldMatrix = MakeAffineMatrix(scale, rotate, translate);

		// 2. View行列の生成 (カメラ位置からビュー行列を生成。今回は簡易的にInverseを利用)
		Matrix4x4 cameraMatrix = MakeTranslateMatrix(cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);

		// 3. Projection行列の生成 (透視投影)
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, 1280.0f / 720.0f, 0.1f, 100.0f);

		// 4. Viewport行列の生成
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, 1280.0f, 720.0f, 0.0f, 1.0f);

		// 行列を一つに合成 (WVP * Viewport)
		Matrix4x4 wvpMatrix = Multiply(Multiply(worldMatrix, viewMatrix), projectionMatrix);
		Matrix4x4 wvpViewportMatrix = Multiply(wvpMatrix, viewportMatrix);

		// ーーー 行列変換を行って頂点をスクリーン座標へ変換 ーーー
		Vector screenVertices[3];
		for (int i = 0; i < 3; ++i) {
			screenVertices[i] = Transform(localVertices[i], wvpViewportMatrix);
		}

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// ーーー 追加：3D行列変換によって計算された座標でポリゴンを描画 ーーー
		Novice::DrawTriangle(
			static_cast<int>(screenVertices[0].x), static_cast<int>(screenVertices[0].y),
			static_cast<int>(screenVertices[1].x), static_cast<int>(screenVertices[1].y),
			static_cast<int>(screenVertices[2].x), static_cast<int>(screenVertices[2].y),
			0xFF0000FF, // 赤色
			FillMode::kFillModeSolid
		);

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
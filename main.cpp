#include <Novice.h>
#include <math.h>
#include <assert.h>
#include <utility> 
#include <imgui.h>

struct Vector {
	float x;
	float y;
	float z;
};

struct Matrix4x4
{
	float m[4][4];
	
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


//行列の加法
Matrix4x4 Add(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = A.m[i][j] + B.m[i][j];
		}
	}
	return result;
}

//行列の減法
Matrix4x4 Subtract(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = A.m[i][j] - B.m[i][j];
		}
	}
	return result;
}

//行列の積
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

//逆行列
Matrix4x4 Inverse(Matrix4x4 m) {
	Matrix4x4 result{};

	// 拡大行列 [A | I]
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

		// 行入れ替え
		if (pivotRow != i) {
			for (int j = 0; j < 8; j++) {
				std::swap(a[i][j], a[pivotRow][j]);
			}
		}

		//逆行列が存在しないチェック
		float pivot = a[i][i];
		assert(fabsf(pivot) > 1e-6f);

		// ピボットを1に
		for (int j = 0; j < 8; j++) {
			a[i][j] /= pivot;
		}

		// 他の行を0に
		for (int k = 0; k < 4; k++) {
			if (k == i) continue;

			float factor = a[k][i];
			for (int j = 0; j < 8; j++) {
				a[k][j] -= factor * a[i][j];
			}
		}
	}

	// 右側が逆行列
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = a[i][j + 4];
		}
	}

	return result;
}

//転置行列
Matrix4x4 Transpose(Matrix4x4 m) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = m.m[j][i];
		}
	}
	return result;
}

//単位行列の作成
Matrix4x4 MakeIdentity() {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		result.m[i][i] = 1.0f;
	}
	return result;
}

//3次元マフィン変換行列
Matrix4x4 MakeAffineMatrix(Vector translation, Vector rotation, Vector scale) {
	Matrix4x4 result = MakeIdentity();
	// スケーリング
	result.m[0][0] = scale.x;
	result.m[1][1] = scale.y;
	result.m[2][2] = scale.z;
	// 回転（オイラー角）
	float cx = cosf(rotation.x);
	float sx = sinf(rotation.x);
	float cy = cosf(rotation.y);
	float sy = sinf(rotation.y);
	float cz = cosf(rotation.z);
	float sz = sinf(rotation.z);
	result.m[0][0] *= cy * cz;
	result.m[0][1] *= -cy * sz;
	result.m[0][2] *= sy;
	result.m[1][0] *= sx * sy * cz + cx * sz;
	result.m[1][1] *= -sx * sy * sz + cx * cz;
	result.m[1][2] *= -sx * cy;
	result.m[2][0] *= -cx * sy * cz + sx * sz;
	result.m[2][1] *= cx * sy * sz + sx * cz;
	result.m[2][2] *= cx * cy;
	// 平行移動
	result.m[3][0] = translation.x;
	result.m[3][1] = translation.y;
	result.m[3][2] = translation.z;
	return result;
}

// 行列ブロック間
static const int kRowHeight = 120;

// セル間
static const int kCellHeight = 20;
static const int kCellWidth = 60;
static const int kColumnWidth = kCellWidth * 4 + 40;
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {

	Novice::ScreenPrintf(x, y, label);

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(
				x + column * kCellWidth,
				y + 20 + row * kCellHeight,
				"%6.2f",
				matrix.m[row][column]
			);
		}
	}
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
		/*Vector v1 = {1.0f, 3.0f, -5.0f};
		Vector v2 = {4.0f, -1.0f, 2.0f};
		float k = 4.0f;

		Vector resultAdd = Add(v1, v2);
		Vector resultSub = Subtract(v1, v2);
		Vector resultMul = Multiply(k, v1);
		float resultDot = Dot(v1, v2);
		float resultLen = Length(v1);
		Vector resultNor = Normalize(v2);


		Matrix4x4 mAdd = Add(m1, m2);
		Matrix4x4 mMultiply = Multiply(m1, m2);
		Matrix4x4 mSubtract = Subtract(m1, m2);

		Matrix4x4 mInverseM1 = Inverse(m1);
		Matrix4x4 mInverseM2 = Inverse(m2);

		Matrix4x4 mTransposeM1 = Transpose(m1);
		Matrix4x4 mTransposeM2 = Transpose(m2);

		Matrix4x4 mIdentity = MakeIdentity();*/


		

		
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		//int y = 0;

		/*VectorScreenPrintf(0, y, resultAdd, "Add");
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

		

	    MatrixScreenPrintf(0, 0, mAdd,       "Add");
		MatrixScreenPrintf(0, kRowHeight, mSubtract, "Subtract");
		MatrixScreenPrintf(0, kRowHeight * 2, mMultiply, "Multiply");
		MatrixScreenPrintf(0, kRowHeight * 3, mInverseM1, "inverseM1");
		MatrixScreenPrintf(0, kRowHeight * 4, mInverseM2, "inverseM2");

		MatrixScreenPrintf(kColumnWidth, 0, mTransposeM1, "transposeM1");
		MatrixScreenPrintf(kColumnWidth, kRowHeight, mTransposeM2, "transposeM2");
		MatrixScreenPrintf(kColumnWidth, kRowHeight * 2, mIdentity, "makeidentity");


		VectorScreenPrintf(0, y, resultNor, "Normalize");*/

		Vector Scale{ 1.2f, 0.79f, -2.1f };
		Vector Rotation{ 0.5f, 1.0f, 0.3f };
		Vector Translation{ 2.0f, -1.0f, 3.0f };
		Matrix4x4 affineMatrix = MakeAffineMatrix(Translation, Rotation, Scale);	
		
		MatrixScreenPrintf(0, 0, affineMatrix, "affineMatrix");
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

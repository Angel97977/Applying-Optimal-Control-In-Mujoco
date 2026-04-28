import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.datasets import fetch_california_housing
from sklearn.model_selection import train_test_split, RepeatedKFold, GridSearchCV
from sklearn.preprocessing import StandardScaler, PolynomialFeatures
from sklearn.linear_model import Ridge, RidgeCV
from sklearn.pipeline import Pipeline
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
from sklearn.feature_selection import SelectKBest, f_regression

# 1. CARGA TUS DATOS AQUÍ
# Reemplaza esto con pd.read_csv('tu_archivo.csv')
X, y = fetch_california_housing(return_X_y=True, as_frame=True)
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

# 2. PIPELINE MAMADÍSIMO
pipe = Pipeline([
    ('scaler', StandardScaler()), # Escala obligatorio para Ridge
    ('poly', PolynomialFeatures(degree=2, include_bias=False)), # Interacciones opcionales
    ('select', SelectKBest(score_func=f_regression, k='all')), # Selección features
    ('ridge', Ridge(random_state=42))
])

# 3. GRID PARA TUNEAR TODO
param_grid = {
    'poly__degree': [1, 2], # 1 = lineal, 2 = cuadrático + interacciones
    'select__k': [5, 8, 'all'], # Cuántas features usar
    'ridge__alpha': np.logspace(-4, 4, 100) # 0.0001 a 10000
}

cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=42)

grid = GridSearchCV(
    pipe,
    param_grid,
    cv=cv,
    scoring='neg_root_mean_squared_error',
    n_jobs=-1,
    verbose=1,
    return_train_score=True
)

grid.fit(X_train, y_train)

# 4. RESULTADOS
best_model = grid.best_estimator_
y_pred = best_model.predict(X_test)

print("=== MEJORES PARÁMETROS ===")
print(grid.best_params_)
print(f"\nMejor RMSE CV: {-grid.best_score_:.4f}")
print(f"RMSE Test: {np.sqrt(mean_squared_error(y_test, y_pred)):.4f}")
print(f"MAE Test: {mean_absolute_error(y_test, y_pred):.4f}")
print(f"R2 Test: {r2_score(y_test, y_pred):.4f}")

# 5. CURVA DE VALIDACIÓN - Ver cómo afecta alpha
alphas = np.logspace(-4, 4, 100)
ridge_cv = RidgeCV(alphas=alphas, cv=cv, scoring='neg_mean_squared_error')
X_train_s = StandardScaler().fit_transform(X_train)
ridge_cv.fit(X_train_s, y_train)

plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.semilogx(alphas, ridge_cv.cv_values_.mean(axis=0))
plt.axvline(ridge_cv.alpha_, color='r', linestyle='--', label=f'Mejor alpha: {ridge_cv.alpha_:.3f}')
plt.xlabel('Alpha')
plt.ylabel('MSE promedio CV')
plt.title('Curva de Validación')
plt.legend()

plt.subplot(1,2,2)
plt.scatter(y_test, y_pred, alpha=0.5)
plt.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], 'r--')
plt.xlabel('Real')
plt.ylabel('Predicho')
plt.title('Predicciones vs Real')
plt.tight_layout()
plt.show()

# 6. IMPORTANCIA DE FEATURES
if grid.best_params_['poly__degree'] == 1: # Solo si no usamos poly
    feature_names = X.columns
    coefs = best_model.named_steps['ridge'].coef_
    k_best = grid.best_params_['select__k']
    if k_best!= 'all':
        selected = best_model.named_steps['select'].get_support()
        feature_names = feature_names[selected]

    coef_df = pd.DataFrame({'Feature': feature_names, 'Coef': coefs})
    coef_df['Abs_Coef'] = np.abs(coef_df['Coef'])
    print("\n=== TOP FEATURES ===")
    print(coef_df.sort_values('Abs_Coef', ascending=False).head(10))
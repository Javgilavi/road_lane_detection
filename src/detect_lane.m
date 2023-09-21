%function para el procesado de la imagen y la detección de las lineas mediante Hough
function [line_izq, line_dcha] = detect_lane(f, Q_izq_est, Q_dcha_est)
    
    %Realizo una copia de la imagen en color y escala de grises
    %f_gray = imadjust(rgb2gray(f));
    f_copy = f;

    %% Mascara zona carretera
    [N,M] = size(f(:,:,1));
    mask_heigh = N/2;
    vertex = [M/2, mask_heigh];
    triangle_points = [1,N; M,N; vertex];
    %Mascara triangular mediante el poligono para extraer la carretera de la imagen
    mask2 = poly2mask(triangle_points(:,1), triangle_points(:,2), N, M);
    %Aplico la mascara en las imagenes
    f_copy = bsxfun(@times, f_copy, cast(mask2, 'like', f_copy));
    %f_gray = bsxfun(@times, f_gray, cast(mask2, 'like', f_gray));

    %% Mascara lineas en escala grises
    f_white = whiteMask(f_copy);
    %f_white = f_gray>(quantile(f_gray(:), 0.99)-5);  %quantile me da el valor de casi mayor(0.99) intensidad de la imagen
    %% Mascara lineas amarillas
    f_yellow = yellowMask2(f_copy);
    
    %% Mascara combinada lineas blancas + amarillas
    f_lane = f_white | f_yellow;
    %% Deteccion de bordes
    fBordes = edge(f_lane, 'canny', 0.2);
    fBordes = bwareaopen(fBordes,15);

    %% Separo en parte izq y dcha para separar lineas
    f_izq = fBordes;
    f_dcha = fBordes;
    for i = 1:M
        for j = 1:N
            if(i > M/2)   
                f_izq(j,i) = 0;     %Coloco a 0 todos pixeles zona dcha 
            else
                f_dcha(j,i) = 0;    %Coloco a 0 todos pixeles zona izq
            end
        end
    end
    %% Aplico el elimano de fondo si se uso Kalman
    if(Q_izq_est(1)~=0 && Q_dcha_est(1)~=0)
        f_izq = eliminate_predicted_background(f_izq,M,N,Q_izq_est,30);
        f_dcha = eliminate_predicted_background(f_dcha,M,N,Q_dcha_est,30);
    end

    %% Transformada de Hough para cada linea
    %Izquierda:
    f_izq = center_line(f_izq, "left");
    [H,tabTheta,tabRho] = hough(f_izq);
    %Saco picos mas probables y extraigo sus lineas en la struct lines
    picos_izq = houghpeaks(H, 1,'Threshold',max(H(:))*0.1,'NHoodSize',[51 51]); 
    line_izq = houghlines(f_izq, tabTheta, tabRho, picos_izq, 'FillGap', 4000, 'MinLength', 5);

    %Derecha:
    f_dcha = center_line(f_dcha, "right");
    [H,tabTheta,tabRho] = hough(f_dcha);
    %Saco picos mas probables y extraigo sus lineas en la struct lines
    picos_dcha = houghpeaks(H, 1,'Threshold',max(H(:))*0.1,'NHoodSize',[51 51]); 
    line_dcha = houghlines(f_dcha, tabTheta, tabRho, picos_dcha, 'FillGap', 4000, 'MinLength', 5);

end
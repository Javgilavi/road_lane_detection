function img = eliminate_predicted_background(img, ancho_imagen, alto_imagen, Q_est, margen)
    % elimnate_predicted_background(lane, img, img_width, img_height, top_margin, left_margin, height, Q_est, prediction_error_tolerance) es una función utilizada para eliminar el fondo utilizando la estimación del estado para mejorar el rendimiento. Solo se deja una franja delgada alrededor de la estimación con un ancho igual a la tolerancia de error.
    % Entrada: 
    %   img - matriz numpy.ndarray, la imagen en la que se va a eliminar el fondo
    %   alto_imagen - entero, Altura de la imagen que se va a procesar
    %   Q_est - matriz numpy.ndarray, Estado estimado para la muestra de tiempo actual
    %   margen - entero, la tolerancia en píxeles asignada al estado estimado
    % Salida:
    %   img - matriz numpy.ndarray, la imagen donde el fondo está en negro para mejorar el rendimiento
    rho = Q_est(1);
    theta = Q_est(2); 

    x1 = (rho - alto_imagen * sind(theta)) / cosd(theta); % Coordenada x del punto de inicio de la línea
    y1 = alto_imagen;                                   % Coordenada y del punto de inicio de la línea
    x2 = (rho) / cosd(theta);                            % Coordenada x del punto final de la línea
    y2 = 1;                                             % Coordenada y del punto final de la línea
    
    mascara = zeros(alto_imagen, ancho_imagen);
    mascara = insertShape(mascara, 'Line', [x1, y1, x2, y2], 'LineWidth', margen, 'Color', [1 1 1]);
    mascara = rgb2gray(mascara);

    img = bsxfun(@times, img, cast(mascara, 'like', img));

end




        
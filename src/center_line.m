function imagen_procesada = center_line(imagen, lane)
    [filas, columnas] = size(imagen);
    imagen_procesada = zeros(filas, columnas);
    
    for fila = 1:filas
        % Buscar el primer píxel blanco en la fila actual
        if strcmp(lane, 'right')
            indice_pixel_blanco = find(imagen(fila, :) == 1, 1);
        elseif strcmp(lane, 'left')
            indice_pixel_blanco = find(imagen(fila, :) == 1, 1, 'last');
        end
        
        % Mantener solo el primer píxel blanco y establecer el resto a cero
        if ~isempty(indice_pixel_blanco)
            imagen_procesada(fila, indice_pixel_blanco) = 1;
        end
    end
end
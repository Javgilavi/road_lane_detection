function [imagen_con_lineas, dist_cm] = draw_lanes(f, lanes, colours)
    %{
    entradas:   f - imagen en la que pintar las lineas
                lanes - lista de struct lineas de la carretera. 1 izq || 2 dcha
                colours - lista de dos colores para lineas. 1 izq || 2 dhca

    salidas:    imagen_con_lineas - imagen con lineas y texto ya pintado
                dist_cm - desviación del dentro en centímetros
    %}
        %Variables para obtener intersecciones deseadas
        [N,M] = size(f(:,:,1));
        theta_b = [90 90];
        rho_b = [N 5*N/8];

        %Extraigo las rhos y thetas
        rho = [lanes(1).rho, lanes(2).rho];
        theta = [lanes(1).theta, lanes(2).theta];

        % Copia de la imagen original para evitar modificar la original
        imagen_con_lineas = f;

        % Pintar las líneas en la imagen copiada
        for i = 1:2
           for j = 1:2
               %Interseccion linea con marco inferior y linea horizontal a 1/3 de altura imagen desde abajo
               xIntersec = (rho(i)*sind(theta_b(j))-rho_b(j)*sind(theta(i)))/(cosd(theta(i))*sind(theta_b(j))-sind(theta(i))*cosd(theta_b(j)));
               yIntersec = rho_b(j);
               xy(i,j,:) = [xIntersec, yIntersec];
           end
           imagen_con_lineas = insertShape(imagen_con_lineas, 'Line', [xy(i,1,1), xy(i,1,2), xy(i,2,1), xy(i,2,2)], 'LineWidth', 8, 'Color', colours(i));
        end
        
        %Posición de la linea central
        x = (xy(2,:,1)+xy(1,:,1))/2;
        y = [N;5*N/8];
        
        %Distancia de desviación
        dist_error = (x(1)-M/2); %%Distancia error en pixeles
        dist_metros = dist_error*(3.5/(abs(xy(1,1,1)-xy(2,1,1)))); %%Carretera son 3.5m // hago relación con distancia en pixeles entre carriles
        dist_cm = dist_metros*100;
        
        if (dist_cm > 0)
            texto = sprintf('Desviado %.2fcm IZQUIERDA', dist_cm);
        else
            texto = sprintf('Desviado %.2fcm DERECHA', abs(dist_cm));
        end
        imagen_con_lineas = insertText(imagen_con_lineas, [M/3, N/3], texto, 'FontSize', 20, 'BoxColor', 'black', 'BoxOpacity', 0.6, 'TextColor', 'white');
        imagen_con_lineas = insertShape(imagen_con_lineas, 'Line', [x(1), y(1), x(2), y(2)], 'LineWidth', 5, 'Color', 'yellow');
        imagen_con_lineas = insertShape(imagen_con_lineas, 'Circle', [M/2, N-1, 5], 'LineWidth', 5, 'Color', 'blue');
        
        % Mostrar la imagen con líneas en imshow
        imshow(imagen_con_lineas);
end
        
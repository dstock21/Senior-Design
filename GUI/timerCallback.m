function [] = timerCallback(~,~,guiHandle,hObject)
if ~isempty(guiHandle)
    % get the handles
    handles = guidata(guiHandle);
    if ~isempty(handles)
        while 1
            ba = handles.arduino.BytesAvailable;
            if ba >= 4
                break
            end
        end
        a = fscanf(handles.arduino, '%f');
        
        while (a~= -10000)
            a = fscanf(handles.arduino, '%f');
            continue
        end
        % query your bluetooth board using your code from the while loop
        % if new data, then update the axes
        
        while 1
            ba = handles.arduino.BytesAvailable;
            if ba >= 16
                break
            end
        end
        
        lastwarn('')
        ka = degtorad(fscanf(handles.arduino, '%f'));
        ha = degtorad(fscanf(handles.arduino, '%f'));
        kt = fscanf(handles.arduino, '%f');
        ht = fscanf(handles.arduino, '%f');
        t = handles.t;
        [warnMsg, warnId] = lastwarn;
        if ~isempty(warnMsg)
            ka = NaN;
            ha = NaN;
            kt = NaN;
            ht = NaN;
            t = handles.t;
        end
        
        
        ax = fk([ha; ka; 0]);
        [m, n] = size(ax);
        if m < 2 || n < 2
            disp('Kwasia');
        end
        xdata = [get(handles.Pos(2,1),'XData'), ax(1,2)];
        ydata = [get(handles.Pos(2,1),'YData'), ax(2,2)];
        set(handles.Pos(2,1),'XData',xdata,'YData',ydata);
        
        xdata = [get(handles.Hka,'XData') t];
        ydata = [get(handles.Hka,'YData') ka];
        set(handles.Hka,'XData',xdata,'YData',ydata);
        
        xdata = [get(handles.Hkt,'XData') t];
        ydata = [get(handles.Hkt,'YData') kt];
        set(handles.Hkt,'XData',xdata,'YData',ydata);
        
        xdata = [get(handles.Hha,'XData') t];
        ydata = [get(handles.Hha,'YData') ha];
        set(handles.Hha,'XData',xdata,'YData',ydata);
        
        xdata = [get(handles.Hht,'XData') t];
        ydata = [get(handles.Hht,'YData') ht];
        set(handles.Hht,'XData',xdata,'YData',ydata);
        
        handles.t = t;
        
        set(handles.text4, 'String', num2str(ka));
        set(handles.text6, 'String', num2str(ht));
        set(handles.text7, 'String', num2str(kt));
        set(handles.text8, 'String', num2str(ha));
        
        handles.t = t + 0.1;
        guidata(hObject,handles);
        
    end
end
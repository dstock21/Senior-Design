function [] = timerCallback(~,~,guiHandle,hObject)
if ~isempty(guiHandle)
    % get the handles
    handles = guidata(guiHandle);
    if ~isempty(handles)
        % query your bluetooth board using your code from the while loop
        % if new data, then update the axes
        
        [m, n] = size(handles.values);
        if (handles.time > n)
            stop(handles.timer);
        end
        tym = handles.time + 1;
        ax = handles.values([1,2,3,4,5,6], tym);
        xAll = handles.values([7,8,9,10,11,12,13,14], tym);
        xAllr = handles.values([15,16,17,18,19,20,21,22], tym);
        ka = handles.values(23, tym);
        kt = handles.values(24, tym);
        ha = handles.values(25, tym);
        ht = handles.values(26, tym);
        t = handles.values(27, tym);
        if (handles.curr > 10)
            xpos = get(handles.Pos(2,1),'XData');
            ypos = get(handles.Pos(2,1),'YData');
            xdata = [xpos(end - 10:end), ax(1,2)];
            ydata = [ypos(end - 10:end), ax(2,2)];
            set(handles.Pos(2,1),'XData',xdata,'YData',ydata);
        else
            xpos = get(handles.Pos(2,1),'XData');
            ypos = get(handles.Pos(2,1),'YData');
            xdata = [xpos(end - handles.limit:end), ax(1,2)];
            ydata = [ypos(end - handles.limit:end), ax(2,2)];
            set(handles.Pos(2,1),'XData',xdata,'YData',ydata);
        end
        
        if (handles.curr > handles.limit)
            
            xdata = xAll([1 3 5 7]);
            ydata = xAll([2 4 6 8]);
            set(handles.sim(1,1),'XData',xdata,'YData',ydata);
            xdata = xAll([1 3 5 7]);
            ydata = xAll([2 4 6 8]);
            set(handles.sim(2,1),'XData',xdata,'YData',ydata);
            
            xdata = xAllr([1 3 5 7]);
            ydata = xAllr([2 4 6 8]);
            set(handles.sim(3,1),'XData',xdata,'YData',ydata);
            xdata = xAllr([1 3 5 7]);
            ydata = xAllr([2 4 6 8]);
            set(handles.sim(4,1),'XData',xdata,'YData',ydata);
            
            
            set(handles.Htext1, 'string', handles.HTstr1, 'position', xAll(1:2) + [0.03,0]);
            set(handles.Htext2, 'string', handles.HTstr2, 'position', xAll(3:4)+ [0.03,0]);
            set(handles.Htext3, 'string', handles.HTstr3, 'position', xAll(5:6));
            
            
            xpos = get(handles.Hka,'XData');
            ypos = get(handles.Hka,'YData');
            xdata = [xpos(end - handles.limit:end) t];
            ydata = [ypos(end - handles.limit:end) ka];
            set(handles.Hka,'XData',xdata,'YData',ydata);
            
            
            xpos = get(handles.Hkt,'XData');
            ypos = get(handles.Hkt,'YData');
            xdata = [xpos(end - handles.limit:end) t];
            ydata = [ypos(end - handles.limit:end) kt];
            set(handles.Hkt,'XData',xdata,'YData',ydata);
            
            
            xpos = get(handles.Hha,'XData');
            ypos = get(handles.Hha,'YData');
            xdata = [xpos(end - handles.limit:end) t];
            ydata = [ypos(end - handles.limit:end) ha];
            set(handles.Hha,'XData',xdata,'YData',ydata);
            
            
            xpos = get(handles.Hht,'XData');
            ypos = get(handles.Hht,'YData');
            xdata = [xpos(end - handles.limit:end) t];
            ydata = [ypos(end - handles.limit:end) ht];
            set(handles.Hht,'XData',xdata,'YData',ydata);
        else
            xdata = xAll([1 3 5 7]);
            ydata = xAll([2 4 6 8]);
            set(handles.sim(1,1),'XData',xdata,'YData',ydata);
            xdata = xAll([1 3 5 7]);
            ydata = xAll([2 4 6 8]);
            set(handles.sim(2,1),'XData',xdata,'YData',ydata);
            
            
            set(handles.Htext1, 'string', handles.HTstr1, 'position', xAll(1:2) + [0.03,0]);
            set(handles.Htext2, 'string', handles.HTstr2, 'position', xAll(3:4)+ [0.03,0]);
            set(handles.Htext3, 'string', handles.HTstr3, 'position', xAll(5:6));
            
            
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
        end
        
        handles.time = tym;
        
        set(handles.text4, 'String', num2str(ka));
        set(handles.text6, 'String', num2str(ht));
        set(handles.text7, 'String', num2str(kt));
        set(handles.text8, 'String', num2str(ha));
        guidata(hObject,handles);
    end
end
 function [] = timerCallback(hObject,~,guiHandle)
 if ~isempty(guiHandle)
      % get the handles
      handles = guidata(guiHandle);
      if ~isempty(handles)
          % query your bluetooth board using your code from the while loop
          % if new data, then update the axes
          
          t = handles.time + 1;
          ax(1, 1) = handles.values(t, 1);
          ax(1, 2) = handles.values(t, 2);
          ka = handles.values(t, 3);
          kt = handles.values(t, 5);
          ha = handles.values(t, 7);
          ht = handles.values(t, 9);
          t = handles.values(t, 4);
           xdata = [get(handles.Pos(2,1),'XData'), ax(1,2)];
           ydata = [get(handles.Pos(2,1),'YData'), ax(2,2)];    
           set(handles.Pos(2,1),'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hka,'XData') ka];
           ydata = [get(handles.Hka,'YData') t];    
           set(handles.Hka,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hkt,'XData') kt];
           ydata = [get(handles.Hkt,'YData') t];    
           set(handles.Hkt,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hha,'XData') ha];
           ydata = [get(handles.Hha,'YData') t];    
           set(handles.Hha,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hht,'XData') ht];
           ydata = [get(handles.Hht,'YData') t];    
           set(handles.Hht,'XData',xdata,'YData',ydata);
           
           handles.time = t;
           
           set(handles.text4, 'String', num2str(ka));
           set(handles.text6, 'String', num2str(kt));
           set(handles.text7, 'String', num2str(ha));
           set(handles.text8, 'String', num2str(ht));
           guidata(hObject,handles);
      end
  end
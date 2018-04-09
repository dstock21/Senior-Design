
function [ h ] = stylefig( h, s )
    try
    import java.io.Reader
    import com.steadystate.css.parser.*
    import org.w3c.dom.css.CSSStyleSheet
    import org.w3c.css.sac.InputSource
    if ~ishandle(h)
        h=hgload(h);
    end
    
    special=struct('background',@background);
    
    r = java.io.FileReader(which(s));
    parser = CSSOMParser();
    is = InputSource(r);
    styleSheet = parser.parseStyleSheet(is);

    rules=getCssRules(styleSheet);
    
        for iRule=0:getLength(rules)-1
           rule=item(rules,iRule);
           selector=char(getSelectorText(rule));
           style=getStyle(rule);
           objects=selectobjects(h,selector)';
           for object=objects
               for iStyle=0:getLength(style)-1
                   myStyle=char(item(style,iStyle));
                   
                   if ~isfield(special,myStyle)
                       value=parsevalue(char(getPropertyValue(style,myStyle)));
                       try
                        set(object,myStyle,value);
                       catch
                           e=lasterror;
                           disp(e.message)
                       end
                   else
                       f=special.(myStyle);
                       f(object,char(getPropertyValue(style,myStyle)));
                   end
               end
            end
        end
    catch
        % You're out of luck. No styling today...
    end
function [ value ] = parsevalue( pv )
           if ~isempty(regexp(pv, ' ')) && isempty(regexp(pv, '^".*"$'))
               pv=['[' pv ']'];
           end
           if regexp(pv, '^".*"$')
               if isempty(regexp(pv, '^"\w+"$'))
                   pv=pv(2:length(pv)-1);
               else
                   pv=strrep(pv,'"','''');
               end
           end
           if regexp(pv, '^[^\d][\w]+$')
               pv=['''' pv ''''];
           end

           value=eval(pv);
		   
function [ objects ] = selectobjects(parents,rule)
    objects=[];
    
    % Split comma delimiters
    rules=textscan(rule, '%s', 'delimiter',',');
    rules=rules{:};
    if length(rules) > 1
        for i=1:length(rules)
           objects=[objects;selectobjects(parents,rules{i})];
        end
        return
    else
        %continue
    end
    
    myRule=strtok(rule);
    
    expr = '(?<Type>\w+)\.(?<Style>\w+)';
    
    objects=[objects;findobj(parents,regexp(myRule, '\*?.(?<Style>\w+)', 'names'))];
    objects=[objects;findobj(parents,regexp(myRule, '^(?<Type>\w+)$', 'names'))];
    objects=[objects;findobj(parents,regexp(myRule, '(?<Type>\w+)\.(?<Style>\w+)', 'names'))];
    objects=[objects;findobj(parents,regexp(myRule, '^\*?\#(?<Tag>\w+)$', 'names'))];
    if regexp(myRule, '^\*$')
        objects=[objects;findobj(parents)];
    end

    % Split space delimiters
    rules=textscan(rule, '%s', 'delimiter');
    rules=rules{:};
    if length(rules) > 1
        for r=rules
           objects=selectobjects(objects,join(rules(2:end),' '));
        end
        return
    else
        % continue
    end

function result=join(c1,c2)
    result={};
    for i=1:length(c1)
        result{i}=[c1{i} c2];
    end
    result=[result{:}];
    
function background(object,value)
    result=regexp(value, 'url\([''"]?(?<url>.*?)[''"]?\)', 'names');
    
    i=imread(which(result.url));
    
    u=get(object,'Units');
    set(object,'Units','Pixels');
    p=get(object,'Position');
    
    d=size(i);
    i=imresize(i,min(p(3:4)./d([2 1])));
    set(object,'CData',i);
    set(object,'Units',u);
    

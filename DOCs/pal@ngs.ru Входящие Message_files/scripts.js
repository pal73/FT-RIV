function submit_search( ) {

	if ($('#act').val() == '1') {
		var mode;
		if ( $('#web').attr('checked') ) {
			mode = $('#web').val();
		}
		else if ( $('#city').attr('checked') ) {
			mode = $('#city').val();
		}
		else if ( $('#ngs').attr('checked') ) {
			mode = $('#ngs').val();
		}
		else {
			document.frm.submit();
			return true;
		}

		$('#mode').val(mode);
		$('#q').val($('#search_str').val());

		document.ngs_poisk.submit();
		return false;
	}
	else {
		return true;
	}
}

function clear_search_form() {
	$('#search_str').val('');

	if ( $('#email').attr('checked') ) {
		document.frm.submit();
		return true;
	}

	$('#search_str').focus();
}

function del_folder(n) {
	return confirm ('Вы подтверждаете удаление  папки \''+n+'\'?');
}

function clean_folder(n) {
	return confirm ('Вы подтверждаете удаление всех писем из папки \''+n+'\'?');
}

function del_mess() {
	document.getElementById("butn_del").click();
}

function clickSpamButton( spam ) {

	if (document.getElementById('butn_spam').disabled ) {
		return false;
	}
	if ( spam ) {
		if (!confirm("Спам - это сообщения, присылаемые Вам без Вашего согласия, содержащие, как правило, рекламу, коммерческие предложения, бесcмысленный набор слов или символов. Рассылки и уведомления, полученные с сайтов, на которых Вы зарегистрированы, как правило, не являются спамом.\n\nВы уверены, что выбранные Вами сообщения являются спамом?")) {
			return false;
		}
	}

	document.frm.action = 'spam/';
	document.getElementById('butn_spam').disabled = 'disabled';
	document.frm.submit();
}

function msg_ch(f, ch, n) {
	var i = 0;
	if (f.elements['Msg'].length) {
		for (i = 0; i < f.elements['Msg'].length; i++) {
			f.elements['Msg'][i].checked = ch;
			if (ch == 'checked') {
				f.elements['Msg'][i].nextSibling.value = f.elements['Msg'][i].value;
			}
			else {
				f.elements['Msg'][i].nextSibling.value = '';
			}
		}
	} else {
		f.elements['Msg'].checked = ch;

		if (ch == 'checked') {
			f.elements['Msg'].nextSibling.value = f.elements['Msg'].value;
		}
		else {
			f.elements['Msg'].nextSibling.value = '';
		}
	}
	if (n == 'checkbox')
		f.checkbox1.checked = ch;
	else
		f.checkbox.checked = ch;
	document.getElementById('add_operation').style.display = (ch)?'block':'none';
	document.getElementById('butn_del').disabled = (ch)?'':'disabled';
	document.getElementById('link_del').disabled = (ch)?'':'disabled';
	$('#butn_spam').attr('disabled', (ch)?'':'disabled');
	$('#link_spam').attr('disabled', (ch)?'':'disabled');
}

function msg_ch2(f, ch) {
	var i = 0;
	var t = 0;
	if (f.elements['Msg'].length) {
		for (i = 0; i < f.elements['Msg'].length; i++) {
			if (f.elements['Msg'][i].checked) {
				f.elements['Msg'][i].nextSibling.value = f.elements['Msg'][i].value;
				t = 1;
			}
			else {
				f.elements['Msg'][i].nextSibling.value = '';
			}
		}
	} else {
		if (f.elements['Msg'].checked) {
			f.elements['Msg'].nextSibling.value = f.elements['Msg'].value;
			t = 1;
		}
		else {
			f.elements['Msg'].nextSibling.value = '';
		}
	}
	document.getElementById('add_operation').style.display = (t)?'block':'none';
	document.getElementById('butn_del').disabled = (t)?'':'disabled';
	document.getElementById('link_del').disabled = (t)?'':'disabled';

	$('#butn_spam').attr('disabled', (ch)?'':'disabled');
	$('#link_spam').attr('disabled', (ch)?'':'disabled');

}


/** New message **/
function addinfo() {
	var msg;
	var div = document.getElementById('divaddinfo');
	var a = document.getElementById('addinfo');
	var s = div.style.display;
	if (s == 'block') {
		s = 'none';
		msg = 'показать дополнительные параметры';
	} else {
		s = 'block';
		msg = 'скрыть дополнительные параметры';
	}
	div.style.display = s;
	a.innerText = msg;
	return false;
}

function set_mail (f, o) {
	if (f.Book.selectedIndex >= 0) {
		obj = f.elements[''+o];
		if (obj.value != '') {
			obj.value += ', '+f.Book.options[f.Book.selectedIndex].text;
		} else {
			obj.value = f.Book.options[f.Book.selectedIndex].text;
		}
	}
}

function check(){
	var s=document.getElementById('write').value;
	s=s.replace(/[\uE000-\uF8FF\u0098]+/g,'');
	var coding=document.getElementById('desiredCharset').value;
	var sim_win = /[^\u0001-~а-яА-Я\u0402\u0403\u201A\u0453\u201E\u2026\u2020\u2021\u20AC\u2030\u0409\u2039\u040A\u040C\u040B\u040F\u0452\u2018\u2019\u201C\u201D\u2022\u2013\u2014\u2122\u0459\u203A\u045A\u045C\u045B\u045F\u00A0\u040E\u045E\u0408\u00A4\u0490\u00A6\u00A7\u0401\u00A9\u0404\u00AB\u00AC\u00AE\u0407\u00B0\u00B1\u0406\u0456\u0491\u00B5\u00B6\u00B7\u0451\u2116\u0454\u00BB\u0458\u0405\u0455\u0457]+/;
	var sim_koi = /[^\u0001-~а-яА-Я\u2500\u2502\u250C\u2510\u2514\u2518\u251C\u2524\u252C\u2534\u253C\u2580\u2584\u2588\u258C\u2590\u2591\u2592\u2593\u2320\u25A0\u2219\u221A\u2248\u2264\u2265\u00A0\u2321\u00B0\u00B2\u00B7\u00F7\u2550\u2551\u2552\u0451\u2553\u2554\u2555\u2556\u2557\u2558\u2559\u255A\u255B\u255C\u255D\u255E\u255F\u2560\u2561\u0401\u2562\u2563\u2564\u2565\u2566\u2567\u2568\u2569\u256A\u256B\u256C\u00A9]+/;
	if (coding=='KOI8-R') var regex = sim_koi;
	else if (coding=='windows-1251') var regex = sim_win;
	else  return true;
	if (regex.test(s)){
		var result=Array();
		var n=0;
		while (regex.exec(s)!=null){
			var arr=regex.exec(s);
			var e=new RegExp(arr[0],'g');
			s=s.replace(e,'');
			result[n]=arr[0];
			n=n+1;
		}
		if (result.length==1){
			var mess="Символ: "+result.join(" ")+" не может быть отправлен в кодировке "+document.getElementById('desiredCharset').value+".\n Изменить кодировку письма на Универсальную (utf8) и отправить?";
		}else var mess="Символы: "+result.join(" ")+" не могут быть отправлены в кодировке "+document.getElementById('desiredCharset').value+".\n Изменить кодировку письма на Универсальную (utf8) и отправить?";
		if (confirm(mess)){
			document.getElementById('desiredCharset').value = "utf-8";
			return true;
		}
		return false;
	}

}

/** website **/
function file_ch(f, ch) {
	var i = 0;
	if (f.elements['folder']) {
	if (f.elements['folder'].length) {
		for (i = 0; i < f.elements['folder'].length; i++) {
			f.elements['folder'][i].checked = ch;
                }
	} else {
		f.elements['folder'].checked = ch;
	}
	}
	var i = 0;
	if(f.elements['file']) {
	if (f.elements['file'].length) {
		for (i = 0; i < f.elements['file'].length; i++) {
			f.elements['file'][i].checked = ch;
                }
	} else {
		f.elements['file'].checked = ch;
	}
	}

}

var on_domain = 'ngs.ru.d';

function save_in_cookie( name, value ) {
 	$.cookie( name, value,  {expires: 8640000, domain: on_domain, path: '/'} );
}

$(document).ready( function() {
	$('#search_str').focus();

    //удаляем пустой блок с текстовой рекламой (левая колонка)
    if (jQuery(".js-text-adv-box").length) {
        jQuery(".js-text-adv-box").each(function() {
            if (jQuery(this).find(".js-text-adv-links-box").length && jQuery(this).find(".js-text-adv-links-box").children().length > 1) {
                jQuery(this).show();
            }
        });
    }
})
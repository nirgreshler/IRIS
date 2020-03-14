base_name = fullfile(pwd, 'Graphs');
username = getenv('USERNAME');
username = strrep(username, '.', '');
if strcmp(username, 'Gal')
    username = 'galgreshler';
end
if strcmp(username, 'Nir') || strcmp(username, 'nirgr')
    username = 'nirgreshler';
end
wsl_path = ['/home/' username '/Project/IRIS'];
search_path = [wsl_path, '/debug/app/search_graph'];
base_name_in_wsl = ['/mnt/' lower(strrep(strrep(base_name,':',''),'\','/'))];